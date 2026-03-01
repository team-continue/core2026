#include "target_detector.hpp"

using namespace core_enemy_detection;

/**
 * @brief Construct a new enemy Detection::enemy Detection object
 * 
 */
targetDetector::targetDetector() : 
    Node("target_detector")
    {
        imgSub = image_transport::create_subscription(this, "raw_image", std::bind(&targetDetector::detectEnemy, this, _1), "compressed", rmw_qos_profile_default);
        // coordPub = this->create_publisher<geometry_msgs::msg::PoseArray>("enemy_image_Coords", 10);
        dpInfoPub = this->create_publisher<core_msgs::msg::DamagePanelInfoArray>("damage_panels_infomation", 1);
        // tf = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&targetDetector::changeParameter, this, _1));
        declareParameters();
        // calc = new image2camera_vector_angle::vec2agCalculator(calibrationFilePath);
    }

/**
 * @brief コールバック関数 画像処理してカメラ座標の敵の方角をpulishする
 * 
 * @param msg 受け取ったメッセージ
 */
void targetDetector::detectEnemy(const sensor_msgs::msg::Image::ConstSharedPtr imgMsg){
    timeStamp = cv_bridge::toCvShare(imgMsg, "bgr8")->header.stamp;
    if((this->now() - timeStamp).seconds() > 0.1){
        return;
    }

    resetDamagePanelInfo();

    // subscribeされたイメージ取得
    rawImage = cv_bridge::toCvShare(imgMsg, "bgr8")->image;
    float xRatio = (float)image_size[0] / (float)rawImage.size().width;
    float yRatio = (float)image_size[1] / (float)rawImage.size().height;
    cv::resize(rawImage, Image, cv::Size(image_size[0], image_size[1]), xRatio, yRatio);

    // hsv画像&lab画像に変換
    cv::cvtColor(Image, hsvImage, cv::COLOR_BGR2HSV);
    cv::cvtColor(Image, labImage, cv::COLOR_BGR2Lab);


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

    panelLabelMap.num = cv::connectedComponentsWithStats(
        panelLabelMap.image,
        panelLabelMap.labelMap,
        panelLabelMap.status,
        panelLabelMap.centroids
    );


    // パネルの色の検出物の重点の上下に赤or青の検出物があるか探索
    // あれば、その重点はダメージパネルの座標として認識
    detectDamagePanel();

    auto dpMsg = core_msgs::msg::DamagePanelInfoArray();
    dpMsg.header.stamp = timeStamp;
    dpMsg.array = damagePanels;
    dpInfoPub->publish(dpMsg);

    /*** 確認用 ***/
    publishImage("test_rawImage", Image, "bgr8");
    publishImage("mask_red", ledMaskImage, "mono8");
    publishImage("mask_panel", panelMaskImage, "mono8");
    publishImage("labeled_led", ledLabelMap.image, "mono8");
    publishImage("labeled_panel", panelLabelMap.image, "mono8");
    publishResultImage();
    
}

rcl_interfaces::msg::SetParametersResult targetDetector::changeParameter(const std::vector<rclcpp::Parameter> &parameters){

    auto result = rcl_interfaces::msg::SetParametersResult();
    for(auto &param : parameters){
        std::string name = param.get_name();
        if(name == "enemy"){
            auto paramValue = param.as_integer_array();
            mode = static_cast<int32_t>(paramValue[0]);
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
                std::cout << "aaa" << std::endl;
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
        }else if(name == "panel_lab_range_lower"){
            auto paramValue =  param.as_integer_array();
            if((0 <= (int)paramValue[0] && (int)paramValue[0] < panel_lab_range_upper[0]) || ((0 <= (int)paramValue[1] && (int)paramValue[1] < panel_lab_range_upper[1]) || (0 <= (int)paramValue[2] && (int)paramValue[2] > panel_lab_range_upper[2]))){
                declareIntArray(panel_lab_range_lower, paramValue);
                result.successful = true;
            }else{
                result.successful = false;
            }
        }else if(name == "panel_lab_range_upper"){
            auto paramValue =  param.as_integer_array();
            if((100 >= (int)paramValue[0] && (int)paramValue[0] > panel_lab_range_lower[0]) || (255 >= (int)paramValue[1] && (int)paramValue[1] > panel_lab_range_lower[1]) || (255 >= (int)paramValue[2] && (int)paramValue[2] > panel_lab_range_lower[2])){
                declareIntArray(panel_lab_range_upper, paramValue);
                result.successful = true;
            }else{
                result.successful = false;
            }
        }else if(name == "led_kernel_matrix_size"){
            kernel_for_led = cv::Mat::ones(param.as_integer_array()[0], param.as_integer_array()[1], CV_8U);
            result.successful = true;
        }else if(name == "panel_kernel_matrix_size"){
            kernel_for_panel = cv::Mat::ones(param.as_integer_array()[0], param.as_integer_array()[1], CV_8U);
            result.successful = true;
        }else{
            result.successful = true;
        }
    }
    return result;
}

void targetDetector::resetDamagePanelInfo(){
    damagePanels.clear();
    damagePanels.shrink_to_fit();
}

void targetDetector::extractHsvRange(){
    if(mode == 0){
        // hsv画像に対する検出
        cv::Mat mask1, mask2;
        cv::inRange(hsvImage, red_range_lower1, red_range_upper1, mask1);
        cv::inRange(hsvImage, red_range_lower2, red_range_upper2, mask2);
        cv::bitwise_or(mask1, mask2, ledMaskImage);
        // publishImage("red1", mask1, "mono8");
        // publishImage("red2", mask2, "mono8");

        // lab画像に対する検出
        // cv::inRange(labImage, params["red_lab_range_lower"], params["red_lab_range_upper"], ledMaskImage);
        
    }else{
        cv::inRange(hsvImage, blue_range_lower, blue_range_upper, ledMaskImage);
    }
    // hsv画像に対する検出
    // cv::inRange(hsvImage, params["panel_hsv_range_lower"], params["panel_hsv_range_upper"], panelMaskImage);
    
    // lab画像に対する検出
    cv::Mat dst;
    cv::medianBlur(labImage, dst, 3);
    cv::inRange(dst, panel_lab_range_lower, panel_lab_range_upper, panelMaskImage);
}

void targetDetector::applyMorphology(){
    cv::Mat dst;

    cv::morphologyEx(ledMaskImage, dst, cv::MORPH_OPEN, kernel_for_led);
    cv::morphologyEx(dst, ledLabelMap.image, cv::MORPH_CLOSE, kernel_for_led);

    cv::morphologyEx(panelMaskImage, dst, cv::MORPH_OPEN, kernel_for_panel);
    cv::morphologyEx(dst, panelLabelMap.image, cv::MORPH_CLOSE, kernel_for_panel);
}

bool targetDetector::detectDamagePanel(){
    const cv::Mat& panelCenters = panelLabelMap.centroids;
    const cv::Mat& panelStatus = panelLabelMap.status;
    const cv::Mat& ledCenters = ledLabelMap.centroids;
    bool flag = false;
    int num = 0;
    // std::cout << panelLabelMap.num << std::endl;

    for(int i = 1; i < panelLabelMap.num; i++){
        bool upperFlag = false;
        bool lowerFlag = false;
        
        auto pnlRow = panelStatus.ptr<int>(i);
        int pnlLeft = pnlRow[cv::CC_STAT_LEFT];
        int pnlTop = pnlRow[cv::CC_STAT_TOP];
        int pnlWidth = pnlRow[cv::CC_STAT_WIDTH];
        int pnlHeight = pnlRow[cv::CC_STAT_HEIGHT];

        for(int j = 1; j < ledLabelMap.num; j++){
            auto ledRow = ledCenters.ptr<double>(j);
            double ledPointX = ledRow[0];
            double ledPointY = ledRow[1];

            if(ledPointX >= pnlLeft && ledPointX <= pnlLeft + pnlWidth){
                if(ledPointY >= pnlTop - pnlHeight * 0.2){
                    upperFlag = true;
                }
                if(ledPointY <= pnlTop + pnlHeight + pnlHeight * 0.2){
                    lowerFlag = true;
                }
            }
        }
        
        if(upperFlag && lowerFlag){
            auto panelPoint = panelCenters.ptr<double>(i);
            core_msgs::msg::DamagePanelInfo dp;
            dp.left = pnlLeft;
            dp.top = pnlTop;
            dp.width = pnlWidth;
            dp.height = pnlHeight;
            dp.area = pnlRow[cv::CC_STAT_AREA];
            dp.x = panelPoint[0];
            dp.y = panelPoint[1];

            damagePanels.push_back(dp);
            num++;
        }
    }
    
    if(num > 0){
        flag = true;
    }

    return flag;
}

void targetDetector::publishResultImage(){
    for(auto itr = damagePanels.begin(); itr != damagePanels.end(); itr++){
        cv::rectangle(
            Image, 
            cv::Rect(
                cv::Point(itr->left, itr->top),
                cv::Point(itr->left + itr->width, itr->top + itr->height)
            ),
            cv::Scalar(0, 255, 255),
            2
        );
        cv::circle(
            Image,
            cv::Point(itr->x, itr->y),
            3,
            cv::Scalar(0, 255, 255),
            -1
        );
    }
    publishImage("result", Image, "bgr8");
}


/**
 * @brief 処理過程のイメージをpublishして確認するための関数(cv::imshowが何故か使えなかった...)
 * 
 * @param tn publishするときのtopic名
 * @param image publishするイメージ(openCV)
 * @param encoding エンコード 基本的に"bgr8" or "mono8"
 */
void targetDetector::publishImage(std::string tn, cv::Mat image, std::string encoding){
    if(imgPub.find(tn) == imgPub.end())
        addPublisher(tn);
    img2msg.header.stamp = this->get_clock()->now();
    img2msg.image = image;
    img2msg.encoding = encoding;
    imgMsg = img2msg.toImageMsg();
    imgPub[tn].publish(*imgMsg.get());
}

void targetDetector::addPublisher(std::string tn){
    std::cout << "add publisher : "<< tn << std::endl;
    auto pubName = image_transport::create_publisher(this, tn, rmw_qos_profile_default);
    imgPub[tn] = pubName;
}

void targetDetector::addPublisher(std::vector<std::string> tn){
    for(auto itr = tn.begin(); itr != tn.end(); itr++){
        addPublisher(*itr);
    }
}

void targetDetector::declareParameters(){
    for(auto itr = params.begin(); itr != params.end(); itr++){
        this->declare_parameter(itr->first, itr->second);
    }
    kernel_for_led = cv::Mat::ones(this->get_parameter("led_kernel_matrix_size").as_integer_array()[0], this->get_parameter("led_kernel_matrix_size").as_integer_array()[1], CV_8U);
    kernel_for_panel = cv::Mat::ones(this->get_parameter("panel_kernel_matrix_size").as_integer_array()[0], this->get_parameter("panel_kernel_matrix_size").as_integer_array()[1], CV_8U);
    declareIntArray(red_range_lower1, this->get_parameter("red_range_lower1").as_integer_array());
    declareIntArray(red_range_lower2, this->get_parameter("red_range_lower2").as_integer_array());
    declareIntArray(red_range_upper1, this->get_parameter("red_range_upper1").as_integer_array());
    declareIntArray(red_range_upper2, this->get_parameter("red_range_upper2").as_integer_array());
    declareIntArray(blue_range_lower, this->get_parameter("blue_range_lower").as_integer_array());
    declareIntArray(blue_range_upper, this->get_parameter("blue_range_upper").as_integer_array());
    declareIntArray(panel_lab_range_lower, this->get_parameter("panel_lab_range_lower").as_integer_array());
    declareIntArray(panel_lab_range_upper, this->get_parameter("panel_lab_range_upper").as_integer_array());
}

void targetDetector::declareIntArray(std::vector<int> &var, std::vector<int64_t> param){
    var.assign(param.begin(), param.end());
}