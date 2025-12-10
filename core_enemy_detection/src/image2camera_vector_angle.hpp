#ifndef IMAGE_CALIBRATOR__IMAGE_CALIBRATOR_HPP_
#define IMAGE_CALIBRATOR__IMAGE_CALIBRATOR_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <math.h>

namespace image2camera_vector_angle
{
class vec2agCalculator
{
public:
    vec2agCalculator(std::string);
    bool calculate(std::vector<double>, std::vector<double>&);
    bool calculate(geometry_msgs::msg::Pose, geometry_msgs::msg::Pose&);
    double getAngle_pitch();
    double getAngle_yaw();
    geometry_msgs::msg::Pose getVector();
private:
    cv::FileStorage fp;
    bool flag;
    cv::Mat matrixK;   // カメラ行列
    float k1, k2, p1, p2, k3;   // 歪み係数(k1, k2 , p1, p2, k3)
    double pitch;
    double yaw;
    geometry_msgs::msg::Pose enemyVec;
    
};
}  // namespace image_calibrator
#endif  // IMAGE_CALIBRATOR__IMAGE_CARIBRATOR_HPP_
