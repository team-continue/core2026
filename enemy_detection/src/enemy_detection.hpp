#ifndef ENEMY_DETECTION__ENEMY_DETECTION_HPP_
#define ENEMY_DETECTION__ENEMY_DETECTION_HPP_

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

#include "image2camera_vector_angle.hpp"


using namespace std::placeholders;

namespace enemy_detection
{
class enemyDetection : 
    public rclcpp::Node
    // public std::enable_shared_from_this<enemy_detection::enemyDetection>
{
public:
    enemyDetection();

private:
    int mode;
    image2camera_vector_angle::vec2agCalculator *calc;
    cv::Mat rawImage;

    /* variable for publishing image */
    cv_bridge::CvImage img2msg;
    sensor_msgs::msg::Image::SharedPtr imgMsg;
    // std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pub;
    std::map<std::string, image_transport::Publisher> imgPub;
    std::vector<std::string> topicName = {};

    /*prameter for detect enemy*/
    std::map<std::string, std::vector<int>> paramName = {
        {"image_size", std::vector<int>({620, 480})},
        {"team", std::vector<int>({0})},
        {"red_range_lower1", std::vector<int>({0, 64, 0})},
        {"red_range_upper1", std::vector<int>({0, 0, 5})},
        {"red_range_lower2", std::vector<int>({172, 64, 0})},
        {"red_range_upper2", std::vector<int>({179, 255, 255})},
        {"blue_range_lower", std::vector<int>({105, 64, 255})},
        {"blue_range_upper", std::vector<int>({135, 255, 255})},
        {"black_range_lower", std::vector<int>({0,  0, 0})},
        {"black_range_upper", std::vector<int>({255, 255, 50})},
        {"kernel_matrix_size", std::vector<int>({5, 5})}
    };
    std::string calibrationFilePath = "None";
    cv::Mat kernel;

    image_transport::Subscriber imgSub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr coordPub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf;
    
    void detectEnemy(const sensor_msgs::msg::Image::ConstSharedPtr);
    void publishImage(std::string, cv::Mat, std::string);
    void addPublisher(std::string);
    void addPublisher(std::vector<std::string>);
    void declareParameters();
};
}   // namespace enemy_detection
#endif  // ENEMY_DETECTION__ENEMY_DETECTION_HPP_
