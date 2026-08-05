#include "core_enemy_detection/target_detector.hpp"

using namespace core_enemy_detection;

/**
 * @brief Construct a new enemy Detection::enemy Detection object
 * 
 */
targetDetector::targetDetector() : 
    Node("target_detector")
    {
        imgSub = image_transport::create_subscription(this, "raw_image", std::bind(&targetDetector::detectEnemy, this, _1), "raw", rmw_qos_profile_default);
        colorSub = this->create_subscription<std_msgs::msg::UInt8>("color", 10, std::bind(&targetDetector::changeTarget, this, _1));
        dpInfoPub = this->create_publisher<core_msgs::msg::DamagePanelInfoArray>("damage_panels_infomation", 1);
        parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&targetDetector::changeParameter, this, _1));
        declareParameters();
    }

void targetDetector::changeTarget(const std_msgs::msg::UInt8::SharedPtr msg){
    mode = static_cast<u_int8_t>(msg->data);
    if(mode == 81 || mode == 67 || mode == 17 || mode == 51){
        operate = true;
    }else{
        operate = false;
    }
    return;
}

/**
 * @brief コールバック関数 画像処理してカメラ座標の敵の方角をpulishする
 * 
 * @param msg 受け取ったメッセージ
 */
void targetDetector::detectEnemy(const sensor_msgs::msg::Image::ConstSharedPtr imgMsg){
    resetDamagePanelInfo();
    operate = true;

    // subscribeされたイメージ取得
    rawImage = cv_bridge::toCvShare(imgMsg, "bgr8")->image;
    float xRatio = (float)image_size[0] / (float)rawImage.size().width;
    float yRatio = (float)image_size[1] / (float)rawImage.size().height;
    cv::resize(rawImage, Image, cv::Size(image_size[0], image_size[1]), xRatio, yRatio);

    if(!operate){
        Image.setTo(cv::Scalar(0, 0, 0));
    }

    // hsv画像&lab画像に変換
    cv::cvtColor(Image, hsvImage, cv::COLOR_BGR2HSV);
    // cv::cvtColor(Image, labImage, cv::COLOR_BGR2Lab);


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

    whiteLabelMap.num = cv::connectedComponentsWithStats(
        whiteLabelMap.image,
        whiteLabelMap.labelMap,
        whiteLabelMap.status,
        whiteLabelMap.centroids
    );


    // パネルの色の検出物の重点の上下に赤or青の検出物があるか探索
    // あれば、その重点はダメージパネルの座標として認識
    detectDamagePanel();

    auto dpMsg = core_msgs::msg::DamagePanelInfoArray();
    dpMsg.header.stamp = timeStamp;
    dpMsg.array = damagePanels;
    dpInfoPub->publish(dpMsg);

    /*** 確認用 ***/
    if(debugMode){
        publishImage("test_rawImage", Image, "bgr8");
        publishImage("mask_red", ledMaskImage, "mono8");
        publishImage("mask_white", whiteMaskImage, "mono8");
        publishImage("labeled_led", ledLabelMap.image, "mono8");
        // publishImage("labeled_panel", panelLabelMap.image, "mono8");
        publishResultImage();
    }
      
    return;
}

rcl_interfaces::msg::SetParametersResult targetDetector::changeParameter(const std::vector<rclcpp::Parameter> &parameters){

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
        }else if(name == "white_range_upper"){
            auto paramValue =  param.as_integer_array();
            if((100 >= (int)paramValue[0] && (int)paramValue[0] > white_range_lower[0]) || (255 >= (int)paramValue[1] && (int)paramValue[1] > white_range_lower[1]) || (255 >= (int)paramValue[2] && (int)paramValue[2] > white_range_lower[2])){
                declareIntArray(white_range_upper, paramValue);
                result.successful = true;
            }else{
                result.successful = false;
            }
        }else if(name == "white_range_lower"){
            auto paramValue =  param.as_integer_array();
            if((0 <= (int)paramValue[0] && (int)paramValue[0] < white_range_upper[0]) || ((0 <= (int)paramValue[1] && (int)paramValue[1] < white_range_upper[1]) || (0 <= (int)paramValue[2] && (int)paramValue[2] > white_range_upper[2]))){
                declareIntArray(white_range_lower, paramValue);
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
    dpLabelMap.clear();
    dpLabelMap.shrink_to_fit();
    damagePanels.clear();
    damagePanels.shrink_to_fit();
}

void targetDetector::extractHsvRange(){
    mode = 17;
    if(mode == 67 || mode == 51){
        // hsv画像に対する検出
        cv::Mat mask1, mask2;
        cv::inRange(hsvImage, red_range_lower1, red_range_upper1, mask1);
        cv::inRange(hsvImage, red_range_lower2, red_range_upper2, mask2);
        cv::bitwise_or(mask1, mask2, ledMaskImage);

        cv::inRange(hsvImage, white_range_lower, white_range_upper, whiteMaskImage);
        // publishImage("red1", mask1, "mono8");
        // publishImage("red2", mask2, "mono8");

        // lab画像に対する検出
        // cv::inRange(labImage, params["red_lab_range_lower"], params["red_lab_range_upper"], ledMaskImage);
        
    }else if(mode == 81 || mode == 17){
        cv::inRange(hsvImage, blue_range_lower, blue_range_upper, ledMaskImage);
        cv::inRange(hsvImage, white_range_lower, white_range_upper, whiteMaskImage);
    }else{
        cv::inRange(hsvImage, blue_range_lower, blue_range_upper, ledMaskImage);
        cv::inRange(hsvImage, white_range_lower, white_range_upper, whiteMaskImage);
    }
    // hsv画像に対する検出
    // cv::inRange(hsvImage, params["panel_hsv_range_lower"], params["panel_hsv_range_upper"], panelMaskImage);
    
    // lab画像に対する検出
    // cv::Mat dst;
    // cv::medianBlur(labImage, dst, 3);
    // cv::inRange(dst, panel_lab_range_lower, panel_lab_range_upper, panelMaskImage);
    
    return;
}

void targetDetector::applyMorphology(){
    cv::Mat dst;

    cv::morphologyEx(ledMaskImage, dst, cv::MORPH_CLOSE, kernel_for_led);
    cv::morphologyEx(dst, ledLabelMap.image, cv::MORPH_OPEN, kernel_for_led);

    cv::morphologyEx(whiteMaskImage, dst, cv::MORPH_CLOSE, kernel_for_led);
    cv::morphologyEx(dst, whiteLabelMap.image, cv::MORPH_OPEN, kernel_for_led);

    // cv::morphologyEx(panelMaskImage, dst, cv::MORPH_OPEN, kernel_for_panel);
    // cv::morphologyEx(dst, panelLabelMap.image, cv::MORPH_CLOSE, kernel_for_panel);
}

bool targetDetector::detectDamagePanel(){
    // const cv::Mat& panelCenters = panelLabelMap.centroids;
    // const cv::Mat& panelStatus = panelLabelMap.status;
    // const cv::Mat& ledCenters = ledLabelMap.centroids;
    // bool flag = false;
    // int num = 0;
    // // std::cout << panelLabelMap.num << std::endl;

    // for(int i = 1; i < panelLabelMap.num; i++){
    //     bool upperFlag = false;
    //     bool lowerFlag = false;
        
    //     auto pnlRow = panelStatus.ptr<int>(i);
    //     int pnlLeft = pnlRow[cv::CC_STAT_LEFT];
    //     int pnlTop = pnlRow[cv::CC_STAT_TOP];
    //     int pnlWidth = pnlRow[cv::CC_STAT_WIDTH];
    //     int pnlHeight = pnlRow[cv::CC_STAT_HEIGHT];

    //     for(int j = 1; j < ledLabelMap.num; j++){
    //         auto ledRow = ledCenters.ptr<double>(j);
    //         double ledPointX = ledRow[0];
    //         double ledPointY = ledRow[1];

    //         if(ledPointX >= pnlLeft && ledPointX <= pnlLeft + pnlWidth){
    //             if(ledPointY >= pnlTop - pnlHeight * 0.5 && ledPointY <= pnlTop + pnlHeight * 0.5){
    //                 upperFlag = true;
    //             }
    //             if(ledPointY >= pnlTop + pnlHeight * 0.5 && ledPointY <= pnlTop + pnlHeight * 1.5){
    //                 lowerFlag = true;
    //             }
    //         }
    //     }
        
    //     if(upperFlag && lowerFlag){
    //         auto panelPoint = panelCenters.ptr<double>(i);
    //         core_msgs::msg::DamagePanelInfo dp;
    //         dp.left = pnlLeft;
    //         dp.top = pnlTop;
    //         dp.width = pnlWidth;
    //         dp.height = pnlHeight;
    //         dp.area = pnlRow[cv::CC_STAT_AREA];
    //         dp.x = panelPoint[0];
    //         dp.y = panelPoint[1];

    //         damagePanels.push_back(dp);
    //         num++;
    //     }
    // }

    const cv::Mat& ledCenters = ledLabelMap.centroids;
    const cv::Mat& ledStatus = ledLabelMap.status;
    const cv::Mat& whiteCenters = whiteLabelMap.centroids;
    const cv::Mat& whiteStatus = whiteLabelMap.status;
    bool flag = false;
    int num = 0;

    for(int i = 1; i < whiteLabelMap.num; i++){
        bool dpFlag = false;

        auto whiteStatusRow = whiteStatus.ptr<int>(i);
        auto whiteCentroidRow = whiteCenters.ptr<double>(i);
        int whiteLeft = whiteStatusRow[cv::CC_STAT_LEFT];
        int whiteTop = whiteStatusRow[cv::CC_STAT_TOP];
        int whiteWidth = whiteStatusRow[cv::CC_STAT_WIDTH];
        int whiteHeight = whiteStatusRow[cv::CC_STAT_HEIGHT];
        double whitePointX = whiteCentroidRow[0];
        double whitePointY = whiteCentroidRow[1];

        for(int j = 1; j < ledLabelMap.num; j++){
            auto ledRowCenter = ledCenters.ptr<double>(j);
            double ledPointX = ledRowCenter[0];
            double ledPointY = ledRowCenter[1];
            if(ledPointX > whiteLeft && ledPointX < whiteLeft + whiteWidth){
                if(ledPointY > whiteTop && ledPointY < whiteTop + whiteHeight){
                    dpFlag = true;
                }
            }
        }
        if(dpFlag){
            labeledImage map;
            map.centroids = (cv::Mat_<double>(1, 2) << whitePointX, whitePointY);
            map.status = (cv::Mat_<int>(1, 4) << whiteLeft, whiteTop, whiteWidth, whiteHeight);
            dpLabelMap.push_back(map);
        }
    }
    // std::cout << dpLabelMap.size() << std::endl;

    for(int i = 0; i < (int)dpLabelMap.size(); i++){
        int baseLedLeft = dpLabelMap[i].status.at<int>(0, 0);
        int baseLedTop = dpLabelMap[i].status.at<int>(0, 1);
        int baseLedWidth = dpLabelMap[i].status.at<int>(0, 2);
        int baseLedHeight = dpLabelMap[i].status.at<int>(0, 3);
        double baseLedPointX = dpLabelMap[i].centroids.at<double>(0, 0);
        double baseLedPointY = dpLabelMap[i].centroids.at<double>(0, 1);

        // std::cout << baseLedLeft << " " << baseLedTop << " " << baseLedWidth << " " << baseLedHeight << std::endl;
        // std::cout << baseLedPointX << " " << baseLedPointY << std::endl;

        for(int j = i + 1; j < (int)dpLabelMap.size(); j++){
            double ledPointX = dpLabelMap[j].centroids.at<double>(0, 0);
            double ledPointY = dpLabelMap[j].centroids.at<double>(0, 1);
            int ledTop = dpLabelMap[j].status.at<double>(0, 1);
            if(ledPointX >= baseLedLeft && ledPointX <= baseLedLeft + baseLedWidth){
                double distance = (ledPointX - baseLedPointX) * (ledPointX - baseLedPointX) + (ledPointY - baseLedPointY) * (ledPointY - baseLedPointY);
                if(distance > 10 * 10 || distance < 350 * 350){
                    // std::cout << "aaa" << std::endl;
                    core_msgs::msg::DamagePanelInfo dp;
                    dp.left = baseLedLeft;
                    dp.top = baseLedTop;
                    dp.width = baseLedWidth;
                    dp.height = ledTop - baseLedTop;
                    dp.area = distance;
                    dp.x = (ledPointX + baseLedPointX) / 2.0;
                    dp.y = (ledPointY + baseLedPointY) / 2.0;
                    damagePanels.push_back(dp);
                    num++;
                }
            }
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
            6,
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
    this->declare_parameter<bool>("debug_mode", false);
    debugMode = this->get_parameter("debug_mode").as_bool();
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