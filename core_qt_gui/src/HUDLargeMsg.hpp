#pragma once

#include "CommonDefinition.hpp"
#include "gui_qt/IWidgetComponet.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QWidget>
#include <QLabel>
#include <QtGui>
#include <chrono>

class HUDLargeMsg : public QWidget, public IWidgetComponet {
    Q_OBJECT

    QLabel *label_;
    std::chrono:: system_clock::time_point start_point;

    constexpr static int margin = 10;
    constexpr static float open_time = 0.1f;
    constexpr static float close_time = 0.1f;
    constexpr static QColor solid_line = QColor(81, 247, 205, 255);
    constexpr static QColor background_color = QColor(0,0,0,200);
    constexpr static int ref_upper_x = 50;
    constexpr static int ref_lower_x = 30;
    constexpr static bool blink_pattern[40] = {
        0,0,1,0,1,0,0,0,0,1,
        0,1,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
    };
    
    QBrush background = QBrush(background_color);
    bool is_animation = false;
    int msg_height = 0;
    float progress = 0.0f;
    float duration_time = 0;
    bool blink_state = false;
    int blink_index = 0;
    QImage rendered_label;

    void paintEvent(QPaintEvent* event);
public:
    HUDLargeMsg(QWidget *parent);
    void setMsg(std::string msg, float duration, QColor color = solid_line);
};