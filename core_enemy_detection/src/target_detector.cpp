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
        declareParameters();
        kernelForLed = cv::Mat::ones(paramName["led_kernel_matrix_size"][0], paramName["led_kernel_matrix_size"][1], CV_8U);
        kernelForPanel = cv::Mat::ones(paramName["panel_kernel_matrix_size"][0], paramName["panel_kernel_matrix_size"][1], CV_8U);
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

    // hsv画像&lab画像に変換
    cv::cvtColor(rawImage, hsvImage, cv::COLOR_BGR2HSV);
    cv::cvtColor(rawImage, labImage, cv::COLOR_BGR2Lab);


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
    // publishImage("test_rawImage", rawImage, "bgr8");
    // publishImage("mask_red", ledMaskImage, "mono8");
    // publishImage("mask_panel", panelMaskImage, "mono8");
    // publishImage("labeled_led", ledLabelMap.image, "mono8");
    // publishImage("labeled_panel", panelLabelMap.image, "mono8");
    // publishResultImage();
    
    // std::cout << damagePanels.size() << std::endl;  // 検出したダメパ数確認
    // for(auto i = damagePanels.begin(); i != damagePanels.end(); i++){
    //     std::cout << i->x << " " << i->y << " " << i->area << std::endl;
    // }

    
    // // enemy_selectorにpublish(手動制御アシスト用)
    // geometry_msgs::msg::PoseArray dpPosition;
    // for(int i = 0; i < (int)panelCoord.size(); i++){
    //     geometry_msgs::msg::Pose pose;
    //     pose.position.x = panelCoord[i][0];
    //     pose.position.y = panelCoord[i][1];
    //     pose.position.z = 0.0;
    //     coords.poses.push_back(pose);
    // }
    // coordPub->publish(coords);
 
    // // とりあえずバウンドボックスが一番大きい座標をtfにbroadcast (オートタレット用)
    // int maxIndex;
    // int cnt = 0;
    // int maxPixcel = 0;
    // for(auto itr = panelStats.begin(); itr != panelStats.end(); itr++){
    //     if(maxPixcel < itr->at(cv::CC_STAT_AREA)){
    //         maxPixcel = itr->at(cv::CC_STAT_AREA);
    //         maxIndex = cnt;
    //     }
    //     cnt++;
    // }
    
    
    // /*** 確認用 ***/
    // auto citr = panelCoord[maxIndex].begin();
    // auto pitr = panelStats[maxIndex].begin();
    // cv::circle(rawImage, cv::Point2d((int)citr[0], (int)citr[1]), 10, cv::Scalar(0,255,255), -1);
    // cv::rectangle(rawImage, cv::Point(pitr[0], pitr[1]), cv::Point(pitr[0] + pitr[2], pitr[1] + pitr[3]), cv::Scalar(0, 255, 255));
    // publishImage("debug", rawImage, "bgr8");
    // /*************/

    // // イメージ座標からカメラ画像に変換
    // std::vector<double> enemyPosi;
    // if(calc->calculate(panelCoord[maxIndex], enemyPosi) == false)
    //     return;
    
    // geometry_msgs::msg::TransformStamped stamped;
    // tf2::Matrix3x3 eulerMat;
    // tf2::Quaternion quat;
    // eulerMat.setRPY(0, calc->getAngle_pitch(), calc->getAngle_yaw());
    // eulerMat.getRotation(quat);

    // stamped.header.stamp = this->now();
    // stamped.header.frame_id = "camera1_link";
    // stamped.child_frame_id = "close_enemy_vector";
    // stamped.transform.translation.x = calc->getVector().position.x;  // Z camera
    // stamped.transform.translation.y = calc->getVector().position.y;  // X camera
    // stamped.transform.translation.z = calc->getVector().position.z;  // Y camera
    // stamped.transform.rotation.x = quat.getX();
    // stamped.transform.rotation.y = quat.getY();
    // stamped.transform.rotation.z = quat.getZ();
    // stamped.transform.rotation.w = quat.getW();
    // tf->sendTransform(stamped);

    // geometry_msgs::msg::TransformStamped stampedPose;
    // stampedPose.header.stamp = this->now();
    // stampedPose.child_frame_id = "close_enemy_position";
    // stampedPose.header.frame_id = "camera1_link";
    // stampedPose.transform.translation.x = enemyPosi[0];  // Z camera
    // stampedPose.transform.translation.y = enemyPosi[1];  // X camera
    // stampedPose.transform.translation.z = enemyPosi[2];  // Y camera
    // stampedPose.transform.rotation.x = quat.getX();
    // stampedPose.transform.rotation.y = quat.getY();
    // stampedPose.transform.rotation.z = quat.getZ();
    // stampedPose.transform.rotation.w = quat.getW();
    // tf->sendTransform(stampedPose);

    // // std::cout << "x :" << stampedPose.transform.translation.x << std::endl;
    // // std::cout << "y :" << stampedPose.transform.translation.y << std::endl;
    // // std::cout << "z :" << stampedPose.transform.translation.z << std::endl;

}

void targetDetector::resetDamagePanelInfo(){
    damagePanels.clear();
    damagePanels.shrink_to_fit();
}

void targetDetector::extractHsvRange(){
    if(mode == 0){
        // hsv画像に対する検出
        cv::Mat mask1, mask2;
        cv::inRange(hsvImage, paramName["red_range_lower1"], paramName["red_range_upper1"], mask1);
        cv::inRange(hsvImage, paramName["red_range_lower2"], paramName["red_range_upper2"], mask2);
        cv::bitwise_or(mask1, mask2, ledMaskImage);
        // publishImage("red1", mask1, "mono8");
        // publishImage("red2", mask2, "mono8");

        // lab画像に対する検出
        // cv::inRange(labImage, paramName["red_lab_range_lower"], paramName["red_lab_range_upper"], ledMaskImage);
        
    }else{
        cv::inRange(hsvImage, paramName["blue_range_lower"], paramName["blue_range_upper"], ledMaskImage);
    }
    // hsv画像に対する検出
    // cv::inRange(hsvImage, paramName["panel_hsv_range_lower"], paramName["panel_hsv_range_upper"], panelMaskImage);
    
    // lab画像に対する検出
    cv::Mat dst;
    cv::medianBlur(labImage, dst, 3);
    cv::inRange(dst, paramName["panel_lab_range_lower"], paramName["panel_lab_range_upper"], panelMaskImage);
}

void targetDetector::applyMorphology(){
    cv::Mat dst;

    cv::morphologyEx(ledMaskImage, dst, cv::MORPH_OPEN, kernelForLed);
    cv::morphologyEx(dst, ledLabelMap.image, cv::MORPH_CLOSE, kernelForLed);

    cv::morphologyEx(panelMaskImage, dst, cv::MORPH_OPEN, kernelForPanel);
    cv::morphologyEx(dst, panelLabelMap.image, cv::MORPH_CLOSE, kernelForPanel);
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
            3,
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
    for(auto itr = paramName.begin(); itr != paramName.end(); itr++){
        this->declare_parameter(itr->first, itr->second);
    }
}

