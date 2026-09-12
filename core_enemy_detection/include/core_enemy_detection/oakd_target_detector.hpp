#ifndef CORE_ENEMY_DETECTION__OAKD_TARGET_DETECTOR_HPP
#define CORE_ENEMY_DETECTION__OAKD_TARGET_DETECTOR_HPP

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <core_msgs/msg/detected_panel_info_array.hpp>
#include <core_msgs/msg/damage_panel_info_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "core_enemy_detection_common.hpp"

using namespace std::placeholders;

namespace core_enemy_detection
{

class oakdTargetDetector : public rclcpp::Node
{
public:
    oakdTargetDetector();

private:
    uint8_t mode = 0;
    float screenWidth = 1280.f;
    float screenHeight = 720.f;
    bool operate = false;
    bool debugMode = false;

    std::vector<int> image_size;
    std::vector<int> red_range_lower1 = {0, 100, 125};
    std::vector<int> red_range_lower2 = {175, 100, 125};
    std::vector<int> red_range_upper1 = {10, 255, 255};
    std::vector<int> red_range_upper2 = {180, 255, 255};
    std::vector<int> blue_range_lower = {0, 0, 0};
    std::vector<int> blue_range_upper = {0, 255, 255};
    
    cv::Mat rawImage;
    cv::Mat hsvImage;
    cv::Mat ledMaskImage;
    cv::Mat kernel_for_led;
    labeledImage ledLabelMap;
    
    std::vector<core_msgs::msg::DamagePanelInfo> damagePanels;
    std::vector<core_msgs::msg::DetectedPanelInfo> detectedPanels;
    std::map<std::string, image_transport::Publisher> imgPub;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    
    rclcpp::Subscription<core_msgs::msg::DetectedPanelInfoArray>::SharedPtr panelInfoSub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr colorSub;

    rclcpp::Publisher<core_msgs::msg::DamagePanelInfoArray>::SharedPtr dpInfoPub;

    void changeTarget(const std_msgs::msg::UInt8::SharedPtr);
    void detectEnemy(const core_msgs::msg::DetectedPanelInfoArray::ConstSharedPtr);
    void extractHsvRange();
    void applyMorphology();
    void detectDamagePanel();
    rcl_interfaces::msg::SetParametersResult changeParameter(const std::vector<rclcpp::Parameter>&);
    void publishImage(std::string, cv::Mat, std::string);
    void addPublisher(std::string);
    void addPublisher(std::vector<std::string>);
    void publishResultImage();
    void declareIntArray(std::vector<int>&, std::vector<int64_t>);
    void declareParameters();

};

}


#endif
