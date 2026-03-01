#include "target_selector.hpp"

using namespace core_enemy_detection;
using std::placeholders::_1;

targetSelector::targetSelector() : 
    Node("target_selector")
{
    nodeTimer = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&targetSelector::publishTargetPoint, this));
    dpInfoSub = this->create_subscription<core_msgs::msg::DamagePanelInfoArray>(
        "damage_panels_infomation", 10, std::bind(&targetSelector::selectTarget, this, _1));
    targetPointPub = this->create_publisher<geometry_msgs::msg::PointStamped>("damage_panel_pose", 1);
    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&targetSelector::changeParameter, this, _1));
    this->declare_parameter("image_size", std::vector<int64_t>{1280, 720});
    declareIntArray(imageSize, this->get_parameter("image_size").as_integer_array());
}

void targetSelector::selectTarget(const core_msgs::msg::DamagePanelInfoArray dpMsg){
    timeStamp = dpMsg.header.stamp;
    damagePanels = dpMsg.array;
    // std::cout << (this->now() - timeStamp).seconds() << std::endl;
    if(damagePanels.size() <= 0){
        flag = false;
        return;
    }

    int maxArea = 0;
    for(auto itr = damagePanels.begin(); itr != damagePanels.end(); itr++){
        if(maxArea < itr->area){
            target = *itr;
            maxArea = itr->area;
        }
    }
    flag = true;
    // std::cout << target.x << " " << target.y << " " << maxArea << std::endl;
    
}

void targetSelector::publishTargetPoint(){
    if(flag){
        auto dpPointMsg = geometry_msgs::msg::PointStamped();
        dpPointMsg.header.stamp = timeStamp;
        dpPointMsg.point.x = target.x - imageSize[0] / 2;
        dpPointMsg.point.y = -(target.y - imageSize[1] / 2);
        dpPointMsg.point.z = 0.0;
        targetPointPub->publish(dpPointMsg);
        return;
    }else{
        auto dpPointMsg = geometry_msgs::msg::PointStamped();
        dpPointMsg.header.stamp = timeStamp;
        dpPointMsg.point.x = 0.0;
        dpPointMsg.point.y = 0.0;
        dpPointMsg.point.z = 1.0;
        targetPointPub->publish(dpPointMsg);
        return;
    }
}

rcl_interfaces::msg::SetParametersResult targetSelector::changeParameter(const std::vector<rclcpp::Parameter> &parameters){
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for(const auto &param : parameters){
        declareIntArray(imageSize, param.as_integer_array());
    }
    
    return result;
}

void targetSelector::declareIntArray(std::vector<int> &var, std::vector<int64_t> param){
    var.assign(param.begin(), param.end());
}

//     // gui -> camera座標
//     geometry_msgs::msg::PoseArray posesMsg;
//     for(auto itr = targetCoords.begin(); itr != targetCoords.end(); itr++){
//         posesMsg.poses.push_back(*itr);
//     }
//     coordsPub2ui->publish(posesMsg);

//     // tf -> 変換後の敵座標
//     geometry_msgs::msg::Pose targetPosi;
//     if(!calc->calculate(*ecitr, targetPosi)){
//         // std::cout << "bbb" << std::endl;
//         return;
//     }
//     // std::cout << "x :" << ecitr->position.x << std::endl;
//     // std::cout << "y :" << ecitr->position.y << std::endl;
//     // std::cout << "z :" << ecitr->position.z << std::endl;
//     geometry_msgs::msg::TransformStamped stampedPosition;
//     tf2::Matrix3x3 eulerMat;
//     tf2::Quaternion quat;
//     eulerMat.setRPY(0, calc->getAngle_pitch(), calc->getAngle_yaw());
//     eulerMat.getRotation(quat);
//     stampedPosition.header.stamp = this->now();
//     stampedPosition.header.frame_id = "camera1_link";
//     stampedPosition.child_frame_id = "target_selector_position";
//     stampedPosition.transform.translation.x = targetPosi.position.x;  // Z camera
//     stampedPosition.transform.translation.y = targetPosi.position.y;  // X camera
//     stampedPosition.transform.translation.z = targetPosi.position.z;  // Y camera
//     stampedPosition.transform.rotation.x = quat.getX();
//     stampedPosition.transform.rotation.y = quat.getY();
//     stampedPosition.transform.rotation.z = quat.getZ();
//     stampedPosition.transform.rotation.w = quat.getW();
//     tf->sendTransform(stampedPosition);
    
//     // tf -> ベクトル
//     geometry_msgs::msg::Pose targetVector = calc->getVector();
//     geometry_msgs::msg::TransformStamped stampedVector;
//     stampedVector.header.stamp = this->now();
//     stampedVector.header.frame_id = "camera1_link";
//     stampedVector.child_frame_id = "target_selector_vector";
//     stampedVector.transform.translation.x = targetVector.position.x;  // Z camera
//     stampedVector.transform.translation.y = targetVector.position.y;  // X camera
//     stampedVector.transform.translation.z = targetVector.position.z;  // Y camera
//     stampedVector.transform.rotation.x = quat.getX();
//     stampedVector.transform.rotation.y = quat.getY();
//     stampedVector.transform.rotation.z = quat.getZ();
//     stampedVector.transform.rotation.w = quat.getW();
//     tf->sendTransform(stampedVector);

// }

// void targetSelector::updatetarget(geometry_msgs::msg::PoseArray coordMsg){
//     // メッセージがなければ返す
//     if(coordMsg.poses.size() == 0)
//         return;

//     std::vector<geometry_msgs::msg::Pose> poses;
//     if(targetCoords.size() == 0){    // 現在の敵座標を持ってなければ追加するだけ
//         for(int i = 0; i < (int)coordMsg.poses.size(); i++){
//             if(i == 0)
//                 coordMsg.poses[i].position.z = 1.0;
//             geometry_msgs::msg::Pose pose;
//             pose.position = coordMsg.poses[i].position;
//             poses.push_back(pose);
//         }
//     }else{  // 敵座標の更新(選択された敵座標だけ探索(一番近い奴))
//         for(int i = 0; i < (int)targetCoords.size(); i++){
//             if(targetCoords[i].position.z == 1.0){
//                 double min = std::numeric_limits<double>::max();
//                 int index;
//                 for(int j = 0; j < (int)coordMsg.poses.size(); j++){
//                     double x = targetCoords[i].position.x - coordMsg.poses[j].position.x;
//                     double y = targetCoords[i].position.y - coordMsg.poses[j].position.y;
//                     double dis = x * x + y * y;
//                     if(min > dis){
//                         min = dis;
//                         index = j;
//                     }
//                 }
//                 coordMsg.poses[index].position.z = targetCoords[i].position.z;
//             }
//             poses.push_back(coordMsg.poses[i]);
//         }
//     }
    
//     targetCoords = poses;
//     // iterator更新
//     for(auto itr = targetCoords.begin(); itr != targetCoords.end(); itr++){
//         if(itr->position.z == 1.0){
//             ecitr = itr;
//         }
//     }
// }

// /**
//  * @brief コントローラの入力によって選択している敵を更新
//  * 
//  * @param controllerMsg コントローラの信号(右へ：正, 左へ：負　想定)
//  */
// void targetSelector::updateSelect(std_msgs::msg::Int32 controllerMsg){
//     if(targetCoords.size() == 0)
//         return;
    
//     ecitr->position.z = 0.0;
//     if(controllerMsg.data > 0){
//         if(ecitr != targetCoords.end() - 1)
//             ecitr++;
//     }else if(controllerMsg.data < 0){
//         if(ecitr != targetCoords.begin())
//             ecitr--;
//     }
//     ecitr->position.z = 1.0;
//     return;
// }