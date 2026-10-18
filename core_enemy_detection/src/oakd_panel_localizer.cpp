#include <core_enemy_detection/oakd_panel_localizer.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <stdexcept>

using namespace core_enemy_detection;

oakdPanelLocalizer::oakdPanelLocalizer() : Node("oakd_panel_localizer")
{
    // launchから渡されたモデルを起動時に読み込む。未指定時は従来のモデル。
    const auto package_share = ament_index_cpp::get_package_share_directory("core_enemy_detection");
    const auto default_model = std::filesystem::path(package_share) / "config" / "best.rvc2.tar.xz";
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Path to the DepthAI NN Archive (restart required to change)";
    descriptor.read_only = true;
    const auto model_path = this->declare_parameter<std::string>(
        "model_path", default_model.string(), descriptor);
    if (!std::filesystem::is_regular_file(model_path)) {
        throw std::runtime_error("model_path is not a file: " + model_path);
    }
    RCLCPP_INFO(this->get_logger(), "Loading model: %s", model_path.c_str());
    panelInfoPub = this->create_publisher<core_msgs::msg::DetectedPanelInfoArray>("detected_panel_info", 1);
    // -----------------------------
    // Camera作成
    // -----------------------------
    auto camera = pipeline.create<dai::node::Camera>();
    camera->build();

    // -----------------------------
    // NN Archive読み込み
    // -----------------------------
    dai::NNArchive model(model_path);

    // -----------------------------ll
    // DetectionNetwork作成
    // -----------------------------
    auto detectionNetwork =pipeline.create<dai::node::DetectionNetwork>();

    detectionNetwork->build(camera, model);

    // 古いフレームを溜めない
    detectionNetwork->input.setBlocking(false);

    // まずは低レイテンシ重視で1 thread
    detectionNetwork->setNumInferenceThreads(1);

    // -----------------------------
    // 出力Queue
    // -----------------------------

    // NNが実際に処理した画像
    frameQueue = detectionNetwork->passthrough.createOutputQueue(1, false);

    // 検出結果
    detectionQueue = detectionNetwork->out.createOutputQueue(1, false);

    // -----------------------------
    // Pipeline開始
    // -----------------------------
    pipeline.start();

    timer = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&oakdPanelLocalizer::detectPanel, this));
}

void oakdPanelLocalizer::detectPanel(){

    if(pipeline.isRunning()){

        // -------------------------
        // 画像取得
        // -------------------------
        auto frameMessage = frameQueue->get<dai::ImgFrame>();

        // -----------------------------
        // 検出結果取得
        // -----------------------------
        auto detectionMessage = detectionQueue->get<dai::ImgDetections>();

        if(frameMessage && detectionMessage){
            // 検出結果がある場合は、検出結果を処理する
            processDetection(frameMessage, detectionMessage);
        }
    }

    return;
}

void oakdPanelLocalizer::processDetection(const std::shared_ptr<dai::ImgFrame>& frameMessage, const std::shared_ptr<dai::ImgDetections>& detectionMessage){
    // 検出結果を処理するコードをここに記述
    // 例: 検出結果をROSメッセージに変換してパブリッシュする
    core_msgs::msg::DetectedPanelInfoArray detectedPanelsMsg;
    for(const auto& detection : detectionMessage->detections){
        
        core_msgs::msg::DetectedPanelInfo panelInfoMsg;
        panelInfoMsg.min_x = detection.xmin;
        panelInfoMsg.min_y = detection.ymin;
        panelInfoMsg.max_x = detection.xmax;
        panelInfoMsg.max_y = detection.ymax;

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
