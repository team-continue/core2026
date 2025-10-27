#include "enemy_selector.hpp"

using namespace enemy_selector;
using std::placeholders::_1;

enemySelector::enemySelector() : 
    Node("enemy_selector"),
    ecitr(enemyCoords.begin())
{
    nodeTimer = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&enemySelector::publishDirection, this));
    coordSub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "enemy_image_Coords", 10, std::bind(&enemySelector::updateEnemy, this, _1));
    selectSub = this->create_subscription<std_msgs::msg::Int32>(
        "hogehoge", 10, std::bind(&enemySelector::updateSelect, this, _1));
    coordsPub2ui = this->create_publisher<geometry_msgs::msg::PoseArray>("enemy_poses", 10);
    coordPub = this->create_publisher<geometry_msgs::msg::Pose>("enemy_coords", 10);
    this->declare_parameter<std::string>("calibration_file_path");
    calibrationFilePath = this->get_parameter("calibration_file_path").as_string();
    calc = new image2camera_vector_angle::vec2agCalculator(calibrationFilePath);
    tf = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void enemySelector::publishDirection(){
    if(enemyCoords.size() == 0){
        // std::cout << "aaa" << std::endl;
        return;
    }

    // gui -> camera座標
    geometry_msgs::msg::PoseArray posesMsg;
    for(auto itr = enemyCoords.begin(); itr != enemyCoords.end(); itr++){
        posesMsg.poses.push_back(*itr);
    }
    coordsPub2ui->publish(posesMsg);

    // tf -> 変換後の敵座標
    geometry_msgs::msg::Pose enemyPosi;
    if(!calc->calculate(*ecitr, enemyPosi)){
        // std::cout << "bbb" << std::endl;
        return;
    }
    // std::cout << "x :" << ecitr->position.x << std::endl;
    // std::cout << "y :" << ecitr->position.y << std::endl;
    // std::cout << "z :" << ecitr->position.z << std::endl;
    geometry_msgs::msg::TransformStamped stampedPosition;
    tf2::Matrix3x3 eulerMat;
    tf2::Quaternion quat;
    eulerMat.setRPY(0, calc->getAngle_pitch(), calc->getAngle_yaw());
    eulerMat.getRotation(quat);
    stampedPosition.header.stamp = this->now();
    stampedPosition.header.frame_id = "camera1_link";
    stampedPosition.child_frame_id = "enemy_selector_position";
    stampedPosition.transform.translation.x = enemyPosi.position.x;  // Z camera
    stampedPosition.transform.translation.y = enemyPosi.position.y;  // X camera
    stampedPosition.transform.translation.z = enemyPosi.position.z;  // Y camera
    stampedPosition.transform.rotation.x = quat.getX();
    stampedPosition.transform.rotation.y = quat.getY();
    stampedPosition.transform.rotation.z = quat.getZ();
    stampedPosition.transform.rotation.w = quat.getW();
    tf->sendTransform(stampedPosition);
    
    // tf -> ベクトル
    geometry_msgs::msg::Pose enemyVector = calc->getVector();
    geometry_msgs::msg::TransformStamped stampedVector;
    stampedVector.header.stamp = this->now();
    stampedVector.header.frame_id = "camera1_link";
    stampedVector.child_frame_id = "enemy_selector_vector";
    stampedVector.transform.translation.x = enemyVector.position.x;  // Z camera
    stampedVector.transform.translation.y = enemyVector.position.y;  // X camera
    stampedVector.transform.translation.z = enemyVector.position.z;  // Y camera
    stampedVector.transform.rotation.x = quat.getX();
    stampedVector.transform.rotation.y = quat.getY();
    stampedVector.transform.rotation.z = quat.getZ();
    stampedVector.transform.rotation.w = quat.getW();
    tf->sendTransform(stampedVector);

}

void enemySelector::updateEnemy(geometry_msgs::msg::PoseArray coordMsg){
    // メッセージがなければ返す
    if(coordMsg.poses.size() == 0)
        return;

    std::vector<geometry_msgs::msg::Pose> poses;
    if(enemyCoords.size() == 0){    // 現在の敵座標を持ってなければ追加するだけ
        for(int i = 0; i < (int)coordMsg.poses.size(); i++){
            if(i == 0)
                coordMsg.poses[i].position.z = 1.0;
            geometry_msgs::msg::Pose pose;
            pose.position = coordMsg.poses[i].position;
            poses.push_back(pose);
        }
    }else{  // 敵座標の更新(選択された敵座標だけ探索(一番近い奴))
        for(int i = 0; i < (int)enemyCoords.size(); i++){
            if(enemyCoords[i].position.z == 1.0){
                double min = std::numeric_limits<double>::max();
                int index;
                for(int j = 0; j < (int)coordMsg.poses.size(); j++){
                    double x = enemyCoords[i].position.x - coordMsg.poses[j].position.x;
                    double y = enemyCoords[i].position.y - coordMsg.poses[j].position.y;
                    double dis = x * x + y * y;
                    if(min > dis){
                        min = dis;
                        index = j;
                    }
                }
                coordMsg.poses[index].position.z = enemyCoords[i].position.z;
            }
            poses.push_back(coordMsg.poses[i]);
        }
    }
    
    enemyCoords = poses;
    // iterator更新
    for(auto itr = enemyCoords.begin(); itr != enemyCoords.end(); itr++){
        if(itr->position.z == 1.0){
            ecitr = itr;
        }
    }
}

/**
 * @brief コントローラの入力によって選択している敵を更新
 * 
 * @param controllerMsg コントローラの信号(右へ：正, 左へ：負　想定)
 */
void enemySelector::updateSelect(std_msgs::msg::Int32 controllerMsg){
    if(enemyCoords.size() == 0)
        return;
    
    ecitr->position.z = 0.0;
    if(controllerMsg.data > 0){
        if(ecitr != enemyCoords.end() - 1)
            ecitr++;
    }else if(controllerMsg.data < 0){
        if(ecitr != enemyCoords.begin())
            ecitr--;
    }
    ecitr->position.z = 1.0;
    return;
}