#include "enemy_detection.hpp"

using namespace enemy_detection;

/**
 * @brief Construct a new enemy Detection::enemy Detection object
 * 
 */
enemyDetection::enemyDetection() : 
    Node("enemy_detection")
    {
        imgSub = image_transport::create_subscription(
            this, "raw_image", std::bind(&enemyDetection::detectEnemy, this, _1), "compressed", rmw_qos_profile_default);
        coordPub = this->create_publisher<geometry_msgs::msg::PoseArray>("enemy_image_Coords", 10);
        tf = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        addPublisher(topicName); 
        declareParameters();
        std::cout << this->get_parameter("red_range_upper1") << std::endl;
        kernel = cv::Mat::ones(paramName["kernel_matrix_size"][0], paramName["kernel_matrix_size"][1], CV_8U);
        calc = new image2camera_vector_angle::vec2agCalculator(calibrationFilePath);
    }

/**
 * @brief コールバック関数 画像処理してカメラ座標の敵の方角をpulishする
 * 
 * @param msg 受け取ったメッセージ
 */
void enemyDetection::detectEnemy(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    // subscribeされたイメージ取得
    rawImage = cv_bridge::toCvShare(msg, "bgr8")->image;

    // hsv画像に変換
    cv::Mat hsvImage;
    cv::cvtColor(rawImage, hsvImage, cv::COLOR_BGR2HSV);

    // 色抽出
    cv::Mat mask_rb, mask_b;
    if(mode == 0){
        cv::Mat mask;
        cv::inRange(hsvImage, paramName["red_range_lower1"], paramName["red_range_upper1"], mask_rb);
        cv::inRange(hsvImage, paramName["red_range_lower2"], paramName["red_range_upper2"], mask);
        mask_rb = mask_rb | mask;
        // publishImage("mask_image_red", mask_rb, "mono8");
    }else{
        cv::inRange(hsvImage, paramName["blue_range_lower"], paramName["blue_range_upper"], mask_rb);
        // publishImage("mask_image_blue", mask_rb, "mono8");
    }
    cv::inRange(hsvImage, paramName["black_range_lower"], paramName["black_range_upper"], mask_b);
    publishImage("mask_image_red_blue", mask_rb, "mono8");
    publishImage("mask_image_black", mask_b, "mono8");

    // ノイズ除去
    cv::Mat dstImage_b, dstImage_rb, dst;
    cv::morphologyEx(mask_b, dst, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(dst, dstImage_b, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask_rb, dst, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(dst, dstImage_rb, cv::MORPH_CLOSE, kernel);
    publishImage("dst_image_black", dstImage_b, "mono8");
    publishImage("dst_image_red_blue", dstImage_rb, "mono8");

    cv::Mat labelImage;
    // ラベリング処理
    cv::Mat statsBlack;
    cv::Mat centroidsBlack;
    cv::Mat statsRed;
    cv::Mat centroidsRed;
    int nLabBlack = cv::connectedComponentsWithStats(dstImage_b, labelImage, statsBlack, centroidsBlack);
    int nLabRed = cv::connectedComponentsWithStats(dstImage_rb, labelImage, statsRed, centroidsRed);

    // 黒の検出物の重点の上下に赤or青の検出物があるか探索
    // あれば、その重点はダメージパネルの座標として認識
    std::vector<std::vector<double>> panelCoord; // パネルの重心座標
    std::vector<std::vector<int>> panelStats; // パネルのステータス
    for(int i = 1; i < nLabBlack; i++){
        bool upperFlag = false;
        bool lowerFlag = false;
        auto cbptr = centroidsBlack.ptr<double>(i);
        auto sbptr = statsBlack.ptr<int>(i);
        for(int j = 1; j < nLabRed; j++){
            auto crptr = centroidsRed.ptr<double>(j);
            if(sbptr[cv::CC_STAT_LEFT] < crptr[0] && sbptr[cv::CC_STAT_LEFT] + sbptr[cv::CC_STAT_WIDTH] > crptr[0]){
                if(cbptr[1] < crptr[1])
                    upperFlag = true;
                else
                    lowerFlag = true;
            }
        }
        if(upperFlag && lowerFlag){
            // std::cout << cbptr[0] << "    " << cbptr[1] << std::endla;
            std::vector<double> centroid_ = {cbptr[0], cbptr[1]};
            panelCoord.push_back(centroid_);
            std::vector<int> stats_ = {sbptr[0], sbptr[1], sbptr[2], sbptr[3], sbptr[4]};
            panelStats.push_back(stats_);
            upperFlag = false;
            lowerFlag = false;
        }
    }

    // 認識数確認
    std::cout << (int)panelCoord.size() << std::endl;
    if(panelCoord.empty() || panelStats.empty())
        return;

    // enemy_selectorにpublish(手動制御アシスト用)
    geometry_msgs::msg::PoseArray coords;
    for(int i = 0; i < (int)panelCoord.size(); i++){
        geometry_msgs::msg::Pose pose;
        pose.position.x = panelCoord[i][0];
        pose.position.y = panelCoord[i][1];
        pose.position.z = 0.0;
        coords.poses.push_back(pose);
    }
    coordPub->publish(coords);
 
    // とりあえずバウンドボックスが一番大きい座標をtfにbroadcast (オートタレット用)
    int maxIndex;
    int cnt = 0;
    int maxPixcel = 0;
    for(auto itr = panelStats.begin(); itr != panelStats.end(); itr++){
        if(maxPixcel < itr->at(cv::CC_STAT_AREA)){
            maxPixcel = itr->at(cv::CC_STAT_AREA);
            maxIndex = cnt;
        }
        cnt++;
    }
    
    
    /*** 確認用 ***/
    auto citr = panelCoord[maxIndex].begin();
    auto pitr = panelStats[maxIndex].begin();
    cv::circle(rawImage, cv::Point2d((int)citr[0], (int)citr[1]), 10, cv::Scalar(0,255,255), -1);
    cv::rectangle(rawImage, cv::Point(pitr[0], pitr[1]), cv::Point(pitr[0] + pitr[2], pitr[1] + pitr[3]), cv::Scalar(0, 255, 255));
    publishImage("aaaa", rawImage, "bgr8");
    /*************/

    // イメージ座標からカメラ画像に変換
    std::vector<double> enemyPosi;
    if(calc->calculate(panelCoord[maxIndex], enemyPosi) == false)
        return;
    
    geometry_msgs::msg::TransformStamped stamped;
    tf2::Matrix3x3 eulerMat;
    tf2::Quaternion quat;
    // std::cout << "yaw : " << calc->getAngle_yaw() << " [rad]" << std::endl;
    // std::cout << "pitch : " << calc->getAngle_pitch() << " [rad]" << std::endl;
    eulerMat.setRPY(0, calc->getAngle_pitch(), calc->getAngle_yaw());
    eulerMat.getRotation(quat);

    stamped.header.stamp = this->now();
    stamped.header.frame_id = "camera1_link";
    stamped.child_frame_id = "close_enemy_vector";
    stamped.transform.translation.x = calc->getVector().position.x;  // Z camera
    stamped.transform.translation.y = calc->getVector().position.y;  // X camera
    stamped.transform.translation.z = calc->getVector().position.z;  // Y camera
    stamped.transform.rotation.x = quat.getX();
    stamped.transform.rotation.y = quat.getY();
    stamped.transform.rotation.z = quat.getZ();
    stamped.transform.rotation.w = quat.getW();
    tf->sendTransform(stamped);

    geometry_msgs::msg::TransformStamped stampedPose;
    stampedPose.header.stamp = this->now();
    stampedPose.child_frame_id = "close_enemy_position";
    stampedPose.header.frame_id = "camera1_link";
    stampedPose.transform.translation.x = enemyPosi[0];  // Z camera
    stampedPose.transform.translation.y = enemyPosi[1];  // X camera
    stampedPose.transform.translation.z = enemyPosi[2];  // Y camera
    stampedPose.transform.rotation.x = quat.getX();
    stampedPose.transform.rotation.y = quat.getY();
    stampedPose.transform.rotation.z = quat.getZ();
    stampedPose.transform.rotation.w = quat.getW();
    tf->sendTransform(stampedPose);

    // std::cout << "x :" << stampedPose.transform.translation.x << std::endl;
    // std::cout << "y :" << stampedPose.transform.translation.y << std::endl;
    // std::cout << "z :" << stampedPose.transform.translation.z << std::endl;

}

/**
 * @brief 処理過程のイメージをpublishして確認するための関数(cv::imshowが何故か使えなかった...)
 * 
 * @param tn publishするときのtopic名
 * @param image publishするイメージ(openCV)
 * @param encoding エンコード 基本的に"bgr8" or "mono8"
 */
void enemyDetection::publishImage(std::string tn, cv::Mat image, std::string encoding){
    if(imgPub.find(tn) == imgPub.end())
        addPublisher(tn);
    img2msg.header.stamp = this->get_clock()->now();
    img2msg.image = image;
    img2msg.encoding = encoding;
    imgMsg = img2msg.toImageMsg();
    imgPub[tn].publish(*imgMsg.get());
}

void enemyDetection::addPublisher(std::string tn){
    std::cout << "add publisher : "<< tn << std::endl;
    auto pubName = image_transport::create_publisher(this, tn, rmw_qos_profile_default);
    imgPub[tn] = pubName;
}

void enemyDetection::addPublisher(std::vector<std::string> tn){
    for(auto itr = tn.begin(); itr != tn.end(); itr++){
        addPublisher(*itr);
    }
}

void enemyDetection::declareParameters(){
    for(auto itr = paramName.begin(); itr != paramName.end(); itr++){
        this->declare_parameter(itr->first, itr->second);
        auto param = this->get_parameter(itr->first).as_integer_array();
        itr->second.clear();
        for(auto pitr = param.begin(); pitr != param.end(); pitr++)
            itr->second.push_back(*pitr);
    }

    this->declare_parameter<std::string>("calibration_file_path");
    calibrationFilePath = this->get_parameter("calibration_file_path").as_string();
    std::cout << calibrationFilePath << std::endl;
}

