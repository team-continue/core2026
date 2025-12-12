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

#include "image2camera_vector_angle.hpp"

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
    image2camera_vector_angle::vec2agCalculator *calc;
    cv::Mat rawImage;
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

    /*prameter for detect enemy*/
    std::map<std::string, std::vector<int>> paramName = {
        {"image_size", std::vector<int>({620, 480})},
        {"team", std::vector<int>({0})},
        {"red_range_lower1", std::vector<int>({0, 125, 125})},
        {"red_range_upper1", std::vector<int>({10, 255, 255})},
        {"red_range_lower2", std::vector<int>({175, 125, 125})},
        {"red_range_upper2", std::vector<int>({180, 255, 255})},
        {"red_lab_range_lower", std::vector<int>({230, 0, 105})},
        {"red_lab_range_upper", std::vector<int>({255, 255, 145})},
        {"blue_range_lower", std::vector<int>({105, 64, 255})},
        {"blue_range_upper", std::vector<int>({135, 255, 255})},
        {"panel_hsv_range_lower", std::vector<int>({75,  70, 55})},
        {"panel_hsv_range_upper", std::vector<int>({100, 170, 95})},
        {"panel_lab_range_lower", std::vector<int>({60, 110, 110})},
        {"panel_lab_range_upper", std::vector<int>({75, 140, 140})},
        {"led_kernel_matrix_size", std::vector<int>({5, 5})},
        {"panel_kernel_matrix_size", std::vector<int>({20, 20})}
    };
    std::string calibrationFilePath = "None";
    cv::Mat kernelForLed;
    cv::Mat kernelForPanel;

    image_transport::Subscriber imgSub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr coordPub;
    rclcpp::Publisher<core_msgs::msg::DamagePanelInfoArray>::SharedPtr dpInfoPub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf;
    
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
};
}   // namespace core_enemy_detection
#endif  // CORE_ENEMY_DETECTION__TARGET_DETECTOR_HPP_
