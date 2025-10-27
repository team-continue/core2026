#include "image2camera_vector_angle.hpp"

using namespace image2camera_vector_angle;


vec2agCalculator::vec2agCalculator(std::string fileName) : 
fp(fileName, cv::FileStorage::READ),
flag(true)
{
    if(!fp.isOpened()){
        std::cout << "Calibration File can not be opened." << std::endl;
        flag = false;
        return;
    }
    fp["K"] >> matrixK;
    cv::Mat matrixD;
    fp["D"] >> matrixD;
    k1 = matrixD.at<float>(0,0);
    k2 = matrixD.at<float>(0,1);
    p1 = matrixD.at<float>(0,2);
    p2 = matrixD.at<float>(0,3);
    k3 = matrixD.at<float>(0,4);
    
    // std::cout << "K : " << matrix_K << std::endl;
    // std::cout << "D : " << matrix_D << std::endl;
}



bool vec2agCalculator::calculate(std::vector<double> imageCoord, std::vector<double> &enemyCoord){
    if(flag == false)
        return flag;

    // (1) ピクセル座標をカメラ座標系に変換
    cv::Mat pixcelCoord = (cv::Mat_<double>(3, 1) << imageCoord[0], imageCoord[1], 1.0); // Z=1
    // std::cout << pixcelCoord.size << std::endl;
    // std::cout << matrixK.size << std::endl;
    // std::cout << pixcelCoord.type() << std::endl;
    // std::cout << pixcelCoord.at<double>(0, 0) << std::endl;
    cv::Mat vec = matrixK.inv() * pixcelCoord;
    enemyVec.position.x = vec.at<double>(2,0);  // イメージ座標のz軸
    enemyVec.position.y = -vec.at<double>(0,0); // イメージ座標のx軸
    enemyVec.position.z = -vec.at<double>(1,0); // イメージ座標のy軸

    // (2) カメラ座標系の方向ベクトルを角度に変換(Θ) 
    yaw = atan2(enemyVec.position.y, enemyVec.position.x);
    pitch = atan2(enemyVec.position.z, enemyVec.position.x);

    // (3) カメラ座標系の敵座標計算
    enemyCoord.resize(3);
    double camera_height = 0.68 + 0.17227; // カメラから機体の中心 + 機体の中心から地面[m]
    double enemy_height = 0.033;  // パネルの高さ(暫定)[m]
    enemyCoord[2] = enemy_height - camera_height;   // camera座標系のz座標
    enemyCoord[0] = enemyCoord[2] / tanf64(pitch); // camrera座標系のx座標
    enemyCoord[1] = enemyCoord[0] * tanf64(yaw);  // camera座標系のy座標

    // (2) 歪み補正(放射歪み, 接線歪み) たぶん使わんくてもいけんやろ
    // float dirX = dir.at<float>(0,0);
    // float dirY = dir.at<float>(1,0);
    // float dirX2 = dirX * dirX;
    // float dirY2 = dirY * dirY;
    // float dis = dirX2 + dirY2;
    // float fixedX = dirX * (1 + dis *(k1 + dis * (k2 + dis * k3)))
    //                 + 2 * p1 * dirX * dirY + p2 * (dis + 2 * dirX2);
    // float fixedY = dirY * (1 + dis *(k1 + dis * (k2 + dis * k3)))
    //                 + p1 * (dis + 2 * dirY2) +  2 * p2 * dirX * dirY;
    
    return flag;
}


bool vec2agCalculator::calculate(geometry_msgs::msg::Pose imageCoord, geometry_msgs::msg::Pose &enemyCoord){
    if(flag == false)
        return flag;
    std::vector<double> pixcelCoord = {imageCoord.position.x, imageCoord.position.y, 0.0};
    std::vector<double> enemyCoord_;
    if(!this->calculate(pixcelCoord, enemyCoord_)){
        // std::cout << "ddd" << std::endl;
        return false;
    }
    enemyCoord.position.x = enemyCoord_[0];
    enemyCoord.position.y = enemyCoord_[1];
    enemyCoord.position.z = enemyCoord_[2];
    return true;
}

/**
 * @brief カメラ座標系でのx軸(pitch)
 * 
 * @return double 
 */
double vec2agCalculator::getAngle_pitch(){
    return pitch;
}

/**
 * @brief カメラ座標系でのY軸回転(yaw角)
 * 
 * @return double 
 */
double vec2agCalculator::getAngle_yaw(){
    return yaw;
}

geometry_msgs::msg::Pose vec2agCalculator::getVector(){
    return enemyVec;
}
