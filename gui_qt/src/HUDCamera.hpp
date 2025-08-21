#pragma once

#include "CommonDefinition.hpp"
#include "gui_qt/IWidgetComponet.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <QWidget>
#include <QtGui>
#include <QLabel>
#include <QImage>
#include <memory>
#include <thread>
#include <mutex>

class EnemyMarker : public QWidget {
    std::vector<geometry_msgs::msg::Pose> enemy_poses_;
public:
    EnemyMarker(QWidget *parent);
    void setMarker(std::vector<geometry_msgs::msg::Pose> poseArray);
    void paintEvent(QPaintEvent*);
};

class HUDCamera : public QWidget, public IWidgetComponet {
    QLabel *label_;
    EnemyMarker *enemy_marker_;
public:
    HUDCamera(QWidget *parent);
    void setEnemyPoses(std::vector<geometry_msgs::msg::Pose> poseArray);
    void setImage(sensor_msgs::msg::CompressedImage::SharedPtr compressedImage);
};