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
#include <QtGui>
#include <sstream>
#include <iomanip>
#include <chrono>

class HUDHPPanel : public QWidget, public IWidgetComponet {
    Q_OBJECT

    QLabel *hp_value_label_;
    QLabel *hp_diff_label_;
    QLabel *hp_label_;
    QProgressBar *hp_bar_;
    QProgressBar *sub_bar_1_;
    QProgressBar *sub_bar_2_;
    QVBoxLayout *layout_;
    QHBoxLayout *hp_layout_;
    QHBoxLayout *sub_layout_;
    QVBoxLayout *sub_bar_layout_;
    std::chrono:: system_clock::time_point start_point_;

    constexpr static QColor solid_ = solid;
    constexpr static QColor solid_red_ = solid_red;
    constexpr static QColor solid_red_another_ = solid_red_another;
    constexpr static QColor translucent_black_ = translucent_black;

    constexpr static float diff_fade_time_ = 0.25f;
    constexpr static float diff_dura_time_ = 0.5f;
    
    float fade_progress_ = 0;
    int max_hp_ = 0;
    int current_hp_ = 0;
    int diff_hp_ = 0;

    void paintEvent(QPaintEvent*);


public:
    HUDHPPanel(QWidget *parent);
    void setMaxHP(int);
    void setHP(int);
};