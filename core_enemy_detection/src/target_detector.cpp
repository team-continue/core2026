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
        declareParameters();
        parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&targetDetector::changeParameter, this, _1));
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

    // subscribeされたイメージ取得
    rawImage = cv_bridge::toCvShare(imgMsg, "bgr8")->image;

    if(!operate){
        rawImage.setTo(cv::Scalar(0, 0, 0));
    }

    // hsv画像&lab画像に変換
    cv::cvtColor(rawImage, hsvImage, cv::COLOR_BGR2HSV);
    // cv::cvtColor(rawImage, labImage, cv::COLOR_BGR2Lab);


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
    if(debugMode){
        publishImage("test_rawImage", rawImage, "bgr8");
        publishImage("mask_red", ledMaskImage, "mono8");
        publishImage("mask_panel", panelMaskImage, "mono8");
        // publishImage("labeled_led", ledLabelMap.image, "mono8");
        // publishImage("labeled_panel", panelLabelMap.image, "mono8");
        publishResultImage();
    }
      
    return;
}

rcl_interfaces::msg::SetParametersResult targetDetector::changeParameter(const std::vector<rclcpp::Parameter> &parameters){

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    // Stage every value first: a rejected request must not partially change state.
    std::map<std::string, std::vector<int>*> ranges = {
        {"red_range_lower1", &red_range_lower1}, {"red_range_upper1", &red_range_upper1},
        {"red_range_lower2", &red_range_lower2}, {"red_range_upper2", &red_range_upper2},
        {"blue_range_lower", &blue_range_lower}, {"blue_range_upper", &blue_range_upper},
        {"panel_range_lower", &panel_range_lower}, {"panel_range_upper", &panel_range_upper}
    };
    std::map<std::string, std::vector<int64_t>> pending;
    for (const auto &param : parameters) {
        const auto &name = param.get_name();
        if (ranges.count(name)) {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
                result.reason = name + " must be an integer array";
                return result;
            }
            const auto values = param.as_integer_array();
            if (values.size() != 3 || values[0] < 0 || values[0] > 179 ||
                values[1] < 0 || values[1] > 255 || values[2] < 0 || values[2] > 255) {
                result.reason = name + " requires H=0..179, S/V=0..255";
                return result;
            }
            pending[name] = values;
        } else if (name == "led_kernel_matrix_size" || name == "panel_kernel_matrix_size") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY ||
                param.as_integer_array().size() != 2 ||
                param.as_integer_array()[0] <= 0 || param.as_integer_array()[1] <= 0) {
                result.reason = name + " requires two positive integers";
                return result;
            }
        } else if (name == "debug_mode" &&
                   param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            result.reason = "debug_mode must be bool";
            return result;
        }
    }
    const std::vector<std::pair<std::string, std::string>> pairs = {
        {"red_range_lower1", "red_range_upper1"}, {"red_range_lower2", "red_range_upper2"},
        {"blue_range_lower", "blue_range_upper"}, {"panel_range_lower", "panel_range_upper"}
    };
    for (const auto &pair : pairs) {
        if (!pending.count(pair.first) && !pending.count(pair.second)) continue;
        for (size_t i = 0; i < 3; ++i) {
            const auto lower = pending.count(pair.first) ? pending.at(pair.first)[i] : ranges.at(pair.first)->at(i);
            const auto upper = pending.count(pair.second) ? pending.at(pair.second)[i] : ranges.at(pair.second)->at(i);
            if (lower > upper) {
                result.reason = pair.first + " must be <= " + pair.second;
                return result;
            }
        }
    }
    for (const auto &entry : pending) {
        declareIntArray(*ranges.at(entry.first), entry.second);
    }
    for (const auto &param : parameters) {
        const auto &name = param.get_name();
        if (name == "debug_mode") debugMode = param.as_bool();
        else if (name == "led_kernel_matrix_size")
            kernel_for_led = cv::Mat::ones(param.as_integer_array()[0], param.as_integer_array()[1], CV_8U);
        else if (name == "panel_kernel_matrix_size")
            kernel_for_panel = cv::Mat::ones(param.as_integer_array()[0], param.as_integer_array()[1], CV_8U);
    }
    result.successful = true;
    return result;
}

void targetDetector::resetDamagePanelInfo(){
    dpLabelMap.clear();
    dpLabelMap.shrink_to_fit();
    damagePanels.clear();
    damagePanels.shrink_to_fit();
}

void targetDetector::extractHsvRange(){
    if(mode == 67 || mode == 51){
        // hsv画像に対する検出 [赤]
        cv::Mat mask1, mask2;
        cv::inRange(hsvImage, red_range_lower1, red_range_upper1, mask1);
        cv::inRange(hsvImage, red_range_lower2, red_range_upper2, mask2);
        cv::bitwise_or(mask1, mask2, ledMaskImage);
        
    }else if(mode == 81 || mode == 17){
        // hsv画像に対する検出 [青]
        cv::inRange(hsvImage, blue_range_lower, blue_range_upper, ledMaskImage);

    }else{
        // どちらでもない場合は適当に処理
        cv::inRange(hsvImage, blue_range_lower, blue_range_upper, ledMaskImage);
    }

    // hsv画像に対する検出 [パネル]
    cv::inRange(hsvImage, panel_range_lower, panel_range_upper, panelMaskImage);
    
    return;
}

void targetDetector::applyMorphology(){
    cv::Mat dst;

    cv::morphologyEx(ledMaskImage, dst, cv::MORPH_CLOSE, kernel_for_led);
    cv::morphologyEx(dst, ledLabelMap.image, cv::MORPH_OPEN, kernel_for_led);

    cv::morphologyEx(panelMaskImage, dst, cv::MORPH_OPEN, kernel_for_panel);
    cv::morphologyEx(dst, panelLabelMap.image, cv::MORPH_CLOSE, kernel_for_panel);
}

void targetDetector::detectDamagePanel(){
    const cv::Mat& panelCenters = panelLabelMap.centroids;
    const cv::Mat& panelStatus = panelLabelMap.status;
    const cv::Mat& ledCenters = ledLabelMap.centroids;
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
                if(ledPointY >= pnlTop - pnlHeight * 0.5 && ledPointY <= pnlTop + pnlHeight * 0.2){
                    upperFlag = true;
                }
                if(ledPointY >= pnlTop + pnlHeight * 0.8 && ledPointY <= pnlTop + pnlHeight * 1.5){
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

    return;
}

void targetDetector::publishResultImage(){
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
    declareIntArray(panel_range_lower, this->get_parameter("panel_range_lower").as_integer_array());
    declareIntArray(panel_range_upper, this->get_parameter("panel_range_upper").as_integer_array());
}

void targetDetector::declareIntArray(std::vector<int> &var, std::vector<int64_t> param){
    var.assign(param.begin(), param.end());
}