#include <core_enemy_detection/oakd_target_detector.hpp>

using namespace core_enemy_detection;

oakdTargetDetector::oakdTargetDetector() : Node("oakd_target_detector")
{
    panelInfoSub = this->create_subscription<core_msgs::msg::DetectedPanelInfoArray>("detected_panel_info", 1, std::bind(&oakdTargetDetector::detectEnemy, this, _1));
    colorSub = this->create_subscription<std_msgs::msg::UInt8>("color", 10, std::bind(&oakdTargetDetector::changeTarget, this, _1));
    dpInfoPub = this->create_publisher<core_msgs::msg::DamagePanelInfoArray>("damage_panels_infomation", 1);
    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&oakdTargetDetector::changeParameter, this, _1));
    declareParameters();
}

void oakdTargetDetector::changeTarget(const std_msgs::msg::UInt8::SharedPtr msg){
    mode = static_cast<uint8_t>(msg->data);
    if(mode == 81 || mode == 67 || mode == 17 || mode == 51){
        operate = true;
    }else{
        operate = false;
    }
    return;
}

void oakdTargetDetector::detectEnemy(const core_msgs::msg::DetectedPanelInfoArray::ConstSharedPtr panelInfoMsg){
    // int num = panelInfoMsg->num;
    detectedPanels = panelInfoMsg->array;
    rawImage = cv_bridge::toCvShare(panelInfoMsg->image, panelInfoMsg, "bgr8")->image;
    
    if(!operate)
        rawImage.setTo(cv::Scalar(0, 0, 0));
    
    // hsv画像に変換
    cv::cvtColor(rawImage, hsvImage, cv::COLOR_BGR2HSV);
    
    // 色抽出
    extractHsvRange();
    
    // ノイズ除去
    applyMorphology();
    
    // ラベリング処理
    ledLabelMap.num = cv::connectedComponentsWithStats(
        ledLabelMap.image, 
        ledLabelMap.labelMap, 
        ledLabelMap.status, 
        ledLabelMap.centroids
    );
    
    // パネルの色の検出物の重点の上下に赤or青の検出物があるか探索
    // あれば、その重点はダメージパネルの座標として認識
    detectDamagePanel();
    // std::cout << "damage panel num: " << damagePanels.size() << std::endl;
    
    auto dpMsg = core_msgs::msg::DamagePanelInfoArray();
    dpMsg.array = damagePanels;
    dpInfoPub->publish(dpMsg);

    // publishImage("raw_image", rawImage, "bgr8");
    // publishResultImage();

    
    return;
}

void oakdTargetDetector::extractHsvRange(){
    if(mode == 67 || mode == 51){
        // hsv画像に対する検出
        cv::Mat mask1, mask2;
        cv::inRange(hsvImage, red_range_lower1, red_range_upper1, mask1);
        cv::inRange(hsvImage, red_range_lower2, red_range_upper2, mask2);
        cv::bitwise_or(mask1, mask2, ledMaskImage);

        // publishImage("red1", mask1, "mono8");
        // publishImage("red2", mask2, "mono8");
        
    }else if(mode == 81 || mode == 17){
        cv::inRange(hsvImage, blue_range_lower, blue_range_upper, ledMaskImage);
    }else{
        cv::inRange(hsvImage, blue_range_lower, blue_range_upper, ledMaskImage);
    }
    
    return;
}

void oakdTargetDetector::applyMorphology(){
    cv::Mat dst;

    cv::morphologyEx(ledMaskImage, dst, cv::MORPH_CLOSE, kernel_for_led);
    cv::morphologyEx(dst, ledLabelMap.image, cv::MORPH_OPEN, kernel_for_led);
    // std::cout << kernel_for_led.size() << std::endl;
    // publishImage("noise_removed", ledLabelMap.image, "mono8");
}

void oakdTargetDetector::detectDamagePanel(){
    damagePanels.clear();
    damagePanels.shrink_to_fit();

    const cv::Mat& ledCenters = ledLabelMap.centroids;
    // std::cout << "detected panel num: " << detectedPanels.size() << std::endl;
    // std::cout << "led num: " << ledLabelMap.num - 1 << std::endl;
    for(auto itr = detectedPanels.begin(); itr != detectedPanels.end(); itr++){
        int count = 0;
        float minX = itr->min_x * rawImage.size().width;
        float maxX = itr->max_x * rawImage.size().width;
        float minY = itr->min_y * rawImage.size().height;
        float maxY = itr->max_y * rawImage.size().height;

        // std::cout << "minX: " << minX << ", maxX: " << maxX << ", minY: " << minY << ", maxY: " << maxY << std::endl;

        for(int i = 1; i < ledLabelMap.num; i++){
            auto ledRow = ledCenters.ptr<double>(i);
            double ledX = ledRow[0];
            double ledY = ledRow[1];
            if(ledX > minX && ledX < maxX && ledY > minY - (maxY - minY) * 0.5 && ledY < maxY + (maxY - minY) * 0.5){
                count++;
            }
        }

        if(count > 1){
            core_msgs::msg::DamagePanelInfo dp;
            dp.left = (int)(minX);
            dp.top = (int)(minY);
            dp.width = (int)(maxX - minX);
            dp.height = (int)(maxY - minY);
            dp.area = (int)(dp.width * dp.height);
            dp.x = (double)(minX + maxX) / 2.f;
            dp.y = (double)(minY + maxY) / 2.f;
            damagePanels.push_back(dp);
        }
    }
}

rcl_interfaces::msg::SetParametersResult oakdTargetDetector::changeParameter(const std::vector<rclcpp::Parameter> &parameters){

    auto result = rcl_interfaces::msg::SetParametersResult();
    for(auto &param : parameters){
        std::string name = param.get_name();
        if(name == "debug_mode"){
            auto paramValue = param.as_bool();
            debugMode = paramValue;
            result.successful = true;
        }else if(name == "image_size"){
            declareIntArray(image_size, param.as_integer_array());
            result.successful = true;
        }else if(name == "red_range_lower1"){
            auto paramValue =  param.as_integer_array();
            if((0 <= (int)paramValue[0] && (int)paramValue[0] < red_range_upper1[0]) || (0 <= (int)paramValue[1] && (int)paramValue[1] < red_range_upper1[1]) || (0 <= (int)paramValue[2] && (int)paramValue[2] < red_range_upper1[2])){
                declareIntArray(red_range_lower1, paramValue);
                result.successful = true;
            }else{
                result.successful = false;
            }
        }else if(name == "red_range_lower2"){
            auto paramValue =  param.as_integer_array();
            if((0 <= (int)paramValue[0] && (int)paramValue[0] < red_range_upper2[0]) || (0 <= (int)paramValue[1] && (int)paramValue[1] < red_range_upper2[1]) || (0 <= (int)paramValue[2] && (int)paramValue[2] < red_range_upper2[2])){
                declareIntArray(red_range_lower2, paramValue);
                result.successful = true;
            }else{
                result.successful = false;
            }
        }else if(name == "red_range_upper1"){
            auto paramValue =  param.as_integer_array();
            if((180 >= (int)paramValue[0] && (int)paramValue[0] > red_range_lower1[0]) || (255 >= (int)paramValue[1] && (int)paramValue[1] > red_range_lower1[1]) || (255 >= (int)paramValue[2] && (int)paramValue[2] > red_range_lower1[2])){
                declareIntArray(red_range_upper1, paramValue);
                result.successful = true;
            }else{
                result.successful = false;
            }
        }else if(name == "red_range_upper2"){
            auto paramValue =  param.as_integer_array();
            if((180 >= (int)paramValue[0] && (int)paramValue[0] > red_range_lower2[0]) ||(255 >= (int)paramValue[1] && (int)paramValue[1] > red_range_lower2[1]) || (255 >= (int)paramValue[2] && (int)paramValue[2] > red_range_lower2[2])){
                declareIntArray(red_range_upper2, paramValue);
                result.successful = true;
            }else{
                result.successful = false;
            }
        }else if(name == "blue_range_lower"){
            auto paramValue =  param.as_integer_array();
            if((0 <= (int)paramValue[0] && (int)paramValue[0] < blue_range_upper[0]) || (0 <= (int)paramValue[1] && (int)paramValue[1] < blue_range_upper[1]) || (0 <= (int)paramValue[2] && (int)paramValue[2] < blue_range_upper[2])){
                declareIntArray(blue_range_lower, paramValue);
                result.successful = true;
            }else{
                result.successful = false;
            }
        }else if(name == "blue_range_upper"){
            auto paramValue =  param.as_integer_array();
            if((180 >= (int)paramValue[0] && (int)paramValue[0] > blue_range_lower[0]) || (255 >= (int)paramValue[1] && (int)paramValue[1] > blue_range_lower[1]) || (255 >= (int)paramValue[2] && (int)paramValue[2] > blue_range_lower[2])){
                declareIntArray(blue_range_upper, paramValue);
                result.successful = true;
            }else{
                result.successful = false;
            }
        }else if(name == "led_kernel_matrix_size"){
            kernel_for_led = cv::Mat::ones(param.as_integer_array()[0], param.as_integer_array()[1], CV_8U);
            result.successful = true;
        }else{
            result.successful = true;
        }
    }
    return result;
}

/**
 * @brief 処理過程のイメージをpublishして確認するための関数(cv::imshowが何故か使えなかった...)
 * 
 * @param tn publishするときのtopic名
 * @param image publishするイメージ(openCV)
 * @param encoding エンコード 基本的に"bgr8" or "mono8"
 */
void oakdTargetDetector::publishImage(std::string tn, cv::Mat image, std::string encoding){
    if(imgPub.find(tn) == imgPub.end())
        addPublisher(tn);

    std_msgs::msg::Header header;
    header.stamp = this->now();

    cv::Mat image_;
    float xRatio = 720.f / (float)image.size().width;
    float yRatio = 720.f / (float)image.size().height;
    cv::resize(image, image_, cv::Size(720, 720), xRatio, yRatio);

    cv::Mat canvas;
    if(encoding == "mono8"){
        canvas = cv::Mat::zeros(cv::Size(screenWidth, screenHeight), CV_8UC1);
    }else{
        canvas = cv::Mat::zeros(cv::Size(screenWidth, screenHeight), CV_8UC3);
    }
    cv::Rect roi((screenWidth - image_.cols) / 2, (screenHeight - image_.rows) / 2, image_.cols, image_.rows);
    image_.copyTo(canvas(roi));
    
    sensor_msgs::msg::Image imgMsg = *cv_bridge::CvImage(header, encoding, canvas).toImageMsg();
    imgPub[tn].publish(imgMsg);
}

void oakdTargetDetector::addPublisher(std::string tn){
    std::cout << "add publisher : "<< tn << std::endl;
    auto pubName = image_transport::create_publisher(this, tn, rmw_qos_profile_default);
    imgPub[tn] = pubName;
}

void oakdTargetDetector::addPublisher(std::vector<std::string> tn){
    for(auto itr = tn.begin(); itr != tn.end(); itr++){
        addPublisher(*itr);
    }
}

void oakdTargetDetector::publishResultImage(){
    for(auto itr = damagePanels.begin(); itr != damagePanels.end(); itr++){
        cv::rectangle(
            rawImage, 
            cv::Rect(
                cv::Point(itr->left, itr->top),
                cv::Point(itr->left + itr->width, itr->top + itr->height)
            ),
            cv::Scalar(0, 255, 255),
            2
        );
        cv::circle(
            rawImage,
            cv::Point(itr->x, itr->y),
            6,
            cv::Scalar(0, 255, 255),
            -1
        );
    }
    publishImage("result", rawImage, "bgr8");
}

void oakdTargetDetector::declareIntArray(std::vector<int>& vec, std::vector<int64_t> paramValue){
    vec.clear();
    for(auto itr = paramValue.begin(); itr != paramValue.end(); itr++){
        vec.push_back(static_cast<int>(*itr));
    }
}

void oakdTargetDetector::declareParameters(){
    this->declare_parameter("debug_mode", debugMode);
    this->declare_parameter("image_size", std::vector<int64_t>({1280, 720}));
    this->declare_parameter("red_range_lower1", std::vector<int64_t>({0, 100, 100}));
    this->declare_parameter("red_range_lower2", std::vector<int64_t>({160, 100, 100}));
    this->declare_parameter("red_range_upper1", std::vector<int64_t>({10, 255, 255}));
    this->declare_parameter("red_range_upper2", std::vector<int64_t>({180, 255, 255}));
    this->declare_parameter("blue_range_lower", std::vector<int64_t>({90, 100, 100}));
    this->declare_parameter("blue_range_upper", std::vector<int64_t>({130, 255, 255}));
    this->declare_parameter("led_kernel_matrix_size", std::vector<int64_t>({1, 1}));
}