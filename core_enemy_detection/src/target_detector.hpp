#ifndef CORE_ENEMY_DETECTION__TARGET_DETECTOR_HPP_
#define CORE_ENEMY_DETECTION__TARGET_DETECTOR_HPP_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <core_msgs/msg/damage_panel_info_array.hpp>

#include "core_enemy_detection_common.hpp"

using namespace std::placeholders;

namespace core_enemy_detection
{

typedef struct labelImage{
    int num = 0;
    cv::Mat image;
    cv::Mat labelMap;
    cv::Mat status;
    cv::Mat centroids;
}labeledImage;

// typedef struct damagePanelInfo{
//     int num;
//     cv::Mat status;
//     cv::Mat centroids;
// }dpInfo;

class targetDetector : 
    public rclcpp::Node
{
public:
    targetDetector();

private:
    int mode = 0;

    cv::Mat rawImage;
    cv::Mat Image;
    cv::Mat hsvImage;
    cv::Mat labImage;
    cv::Mat ledMaskImage;
    cv::Mat panelMaskImage;
    labeledImage ledLabelMap;
    labeledImage panelLabelMap;
    std::vector<core_msgs::msg::DamagePanelInfo> damagePanels;
    rclcpp::Time timeStamp;

    /* variable for publishing image */
    cv_bridge::CvImage img2msg;
    sensor_msgs::msg::Image::SharedPtr imgMsg;
    std::map<std::string, image_transport::Publisher> imgPub;

    /*variable for ros2 parameter*/
    cv::Mat kernel_for_led;
    cv::Mat kernel_for_panel;
    std::vector<int> image_size;
    std::vector<int> red_range_lower1 = {0, 0, 0};
    std::vector<int> red_range_lower2 = {0, 0, 0};
    std::vector<int> red_range_upper1 = {255, 255, 255};
    std::vector<int> red_range_upper2 = {255, 255, 255};
    std::vector<int> blue_range_lower = {0, 0, 0};
    std::vector<int> blue_range_upper = {0, 255, 255};
    std::vector<int> panel_lab_range_lower = {0, 0, 0};
    std::vector<int> panel_lab_range_upper = {0, 255, 255};

    image_transport::Subscriber imgSub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr coordPub;
    rclcpp::Publisher<core_msgs::msg::DamagePanelInfoArray>::SharedPtr dpInfoPub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult changeParameter(const std::vector<rclcpp::Parameter>&);
    
    void detectEnemy(const sensor_msgs::msg::Image::ConstSharedPtr);
    void resetDamagePanelInfo();
    void extractHsvRange();
    void applyMorphology();
    bool detectDamagePanel();
    void publishResultImage();
    void publishImage(std::string, cv::Mat, std::string);

    void addPublisher(std::string);
    void addPublisher(std::vector<std::string>);
    void declareParameters();
    void declareIntArray(std::vector<int>&, std::vector<int64_t>);
};
}   // namespace core_enemy_detection
#endif  // CORE_ENEMY_DETECTION__TARGET_DETECTOR_HPP_
