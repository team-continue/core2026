#include "target_calculator.hpp"

using namespace core_enemy_detection;
using std::placeholders::_1;

targetCalculator::targetCalculator() : Node("target_calculator"){
    dpSub = this->create_subscription<core_msgs::msg::DamagePanelInfoArray>(
        "damage_panels_infomation", 10, std::bind(&targetCalculator::calculateWorldPose, this, _1));
    setCalibrationParam();
}


void targetCalculator::setCalibrationParam(){
    cv::FileStorage fs(fileName, cv::FileStorage::READ);
    if(!fs.isOpened()){
        std::cout << "Calibration File can not be opened." << std::endl;
        return;
    }
    fs["K"] >> matrixK;
    cv::Mat matrixD;
    fs["D"] >> matrixD;
    k1 = matrixD.at<float>(0,0);
    k2 = matrixD.at<float>(0,1);
    p1 = matrixD.at<float>(0,2);
    p2 = matrixD.at<float>(0,3);
    k3 = matrixD.at<float>(0,4);

    return;
}

void targetCalculator::calculateWorldPose(const core_msgs::msg::DamagePanelInfoArray dpMsg){
    dpInfo = dpMsg.array;
    for(auto itr = dpInfo.begin(); itr != dpInfo.end(); itr++){
        // (1) イメージ座標を実座標に変換
        cv::Mat dpPose = (cv::Mat_<double>(3, 1) << itr->x, itr->y, 1.0);
        cv::Mat vec = matrixK.inv() * dpPose;

    }
}