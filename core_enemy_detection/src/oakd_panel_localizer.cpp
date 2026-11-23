#include <core_enemy_detection/oakd_panel_localizer.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <stdexcept>

using namespace core_enemy_detection;

oakdPanelLocalizer::oakdPanelLocalizer() : Node("oakd_panel_localizer")
{
    // モデルパスの既定値を設定
    const auto package_share = ament_index_cpp::get_package_share_directory("core_enemy_detection");
    const auto default_model = std::filesystem::path(package_share) / "config" / "best_yolov8n.rvc2.tar.xz";
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Path to the DepthAI NN Archive (restart required to change)";
    // モデルパスを読み取り専用に設定
    descriptor.read_only = true;
    const auto model_path = this->declare_parameter<std::string>(
        "model_path", default_model.string(), descriptor);
    // モデルファイルの存在を確認
    if (!std::filesystem::is_regular_file(model_path)) {
        throw std::runtime_error("model_path is not a file: " + model_path);
    }
    RCLCPP_INFO(this->get_logger(), "Loading model: %s", model_path.c_str());

    // 画像とパネル情報のPublisherを作成
    panelInfoPub = this->create_publisher<core_msgs::msg::OakdDetectedPanelInfoArray>("detected_panel_info", 1);

    // RGBカメラノードを作成
    auto camera = pipeline.create<dai::node::Camera>();
    // RGBカメラにCAM_Aを指定
    camera->build(dai::CameraBoardSocket::CAM_A);

    // 左右のモノクロカメラを設定
    auto leftCamera = pipeline.create<dai::node::Camera>();
    auto rightCamera = pipeline.create<dai::node::Camera>();
    leftCamera->build(dai::CameraBoardSocket::CAM_B);
    rightCamera->build(dai::CameraBoardSocket::CAM_C);
    
    // 左右画像を640×400・モノクロ・切り抜き・30fpsで出力
    auto leftOutput = leftCamera->requestOutput(
        {640, 400},
        dai::ImgFrame::Type::GRAY8,
        dai::ImgResizeMode::CROP,
        30.0f
    );
    auto rightOutput = rightCamera->requestOutput(
        {640, 400},
        dai::ImgFrame::Type::GRAY8,
        dai::ImgResizeMode::CROP,
        30.0f
    );

    // 深度計算ノードを作成
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    // 左右画像を深度計算ノードに接続
    stereo->build(*leftOutput, *rightOutput);

    // 左右の対応が矛盾する深度を除外
    stereo->setLeftRightCheck(true);
    // 視差を小数画素単位で計算
    stereo->setSubpixel(true);

    // 推論モデルを読み込み
    dai::NNArchive model(model_path);

    // パネル検出と3次元位置計算のノードを作成
    auto detectionNetwork =pipeline.create<dai::node::SpatialDetectionNetwork>();
    // RGBカメラ・深度・モデルを接続
    detectionNetwork->build(camera, stereo, model);

    // 推論待ち画像を最新1件に制限
    detectionNetwork->input.setMaxSize(1);
    detectionNetwork->input.setBlocking(false);

    // 推論スレッド数を1に設定。レイテンシを抑えるため。
    detectionNetwork->setNumInferenceThreads(1);

    // バウンディングボックスのスケールを0.5倍に設定
    detectionNetwork->setBoundingBoxScaleFactor(0.5f);

    // 採用する深度を100～10000mmに制限
    detectionNetwork->setDepthLowerThreshold(100);
    detectionNetwork->setDepthUpperThreshold(10000);

    // ROI内の深度の中央値を使用
    detectionNetwork->setSpatialCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm::MEDIAN);

    // 画像と検出結果の同期ノードを作成
    auto sync = pipeline.create<dai::node::Sync>();

    // 同期する時刻差を1ms以内に設定
    sync->setSyncThreshold(std::chrono::milliseconds(1));
    // 同期できた組だけを出力
    sync->setSyncAttempts(-1);

    // 画像と検出結果を同期入力に接続
    detectionNetwork->passthrough.link(sync->inputs["image"]);
    detectionNetwork->out.link(sync->inputs["detections"]);

    // 受信キューを最新1件に制限
    syncedQueue = sync->out.createOutputQueue(1, false);

    // パイプラインを開始
    pipeline.start();

    // 10msごとに検出結果を確認
    timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&oakdPanelLocalizer::detectPanel, this));
}

void oakdPanelLocalizer::detectPanel(){
    
    
    if(pipeline.isRunning()){
        
        // 推論の同期結果を取得
        auto message = syncedQueue->tryGet<dai::MessageGroup>();
        if(!message){
            return;
        }
        
        // 同期結果からRGB画像と検出結果を取得
        auto frameMessage = message->get<dai::ImgFrame>("image");
        auto detectionMessage = message->get<dai::SpatialImgDetections>("detections");
        if(!frameMessage || !detectionMessage){
            return;
        }

        processDetection(frameMessage, detectionMessage);
    }

    return;
}
// 受け取った検出結果をROS2メッセージに変換してパブリッシュ
void oakdPanelLocalizer::processDetection(const std::shared_ptr<dai::ImgFrame>& frameMessage, const std::shared_ptr<dai::SpatialImgDetections>& detectionMessage){
    core_msgs::msg::OakdDetectedPanelInfoArray detectedPanelsMsg;

    for(const auto& detection : detectionMessage->detections){
        if(detection.confidence < 0.5f){
            continue;
        }
        core_msgs::msg::OakdDetectedPanelInfo panelInfoMsg;
        panelInfoMsg.min_x = detection.xmin;
        panelInfoMsg.min_y = detection.ymin;
        panelInfoMsg.max_x = detection.xmax;
        panelInfoMsg.max_y = detection.ymax;
        panelInfoMsg.position.x = detection.spatialCoordinates.x * 0.001f; // mm to m
        panelInfoMsg.position.y = detection.spatialCoordinates.y * 0.001f;
        panelInfoMsg.position.z = detection.spatialCoordinates.z * 0.001f;

        detectedPanelsMsg.array.push_back(panelInfoMsg);
    }
    detectedPanelsMsg.num = detectionMessage->detections.size();

    cv::Mat frame = frameMessage->getCvFrame();

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "oakd_camera_frame";
    

    detectedPanelsMsg.image = *cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    panelInfoPub->publish(detectedPanelsMsg);
}
