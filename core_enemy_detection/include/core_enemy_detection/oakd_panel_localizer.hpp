#ifndef OAKD_PANEL_LOCALIZER_HPP
#define OAKD_PANEL_LOCALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "core_msgs/msg/detected_panel_info.hpp"
#include "core_msgs/msg/detected_panel_info_array.hpp"

using namespace std::placeholders;


namespace core_enemy_detection
{

class oakdPanelLocalizer : public rclcpp::Node{
    public:
        oakdPanelLocalizer();

    private:
        dai::Pipeline pipeline;
        std::shared_ptr<dai::MessageQueue> frameQueue;
        std::shared_ptr<dai::MessageQueue> detectionQueue;

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<core_msgs::msg::DetectedPanelInfoArray>::SharedPtr panelInfoPub;
        
        void detectPanel();
        void processDetection(const std::shared_ptr<dai::ImgFrame>&, const std::shared_ptr<dai::ImgDetections>&);

};

}
#endif  // OAKD_PANEL_LOCALIZER_HPP
