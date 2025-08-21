#pragma once

#include "CommonDefinition.hpp"
#include "gui_qt/IWidgetComponet.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QWidget>
#include <QLabel>
#include <QImage>
#include <QProgressBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFontDatabase>
#include <sstream>
#include <iomanip>

class HUDAmmoPanel : public QWidget, public IWidgetComponet {
    Q_OBJECT

    QLabel *value_label_;
    QLabel *label_;
    QHBoxLayout *layout_;

    const QColor solid_ = solid;
    const QColor solid_red_ = solid_red;

    void setReload();
public:
    HUDAmmoPanel(QWidget *parent);
    void setValue(int);
};