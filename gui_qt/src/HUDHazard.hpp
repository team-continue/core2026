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

class HUDHazard : public QWidget, public IWidgetComponet {
    QLabel *label_;
    bool state_;
public:
    HUDHazard(QWidget *parent);
    void paintEvent(QPaintEvent*);
    void setState(bool);
};