#pragma once

#include "CommonDefinition.hpp"
#include "gui_qt/IWidgetComponet.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QWidget>
#include <QLabel>
#include <QImage>
#include <QtGui>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <sstream>
#include <iomanip>

class HUDCompass : public QWidget, public IWidgetComponet {
    Q_OBJECT

    // Draw Meter Parameter
    constexpr static int unit_scale = 20;            // でかい目盛の単位
    constexpr static int draw_scale = 10;            // 描画スケール（目盛のpx間隔） [px] 
    constexpr static int scale_size = 8;             // 表示する目盛の長さ [px]
    constexpr static int scale_sub_size = 4;         // 表示するサブ目盛の長さ [px]
    constexpr static int scale_amount = 18;          // 目盛の刻む個数
    constexpr static int scale_sub_amount = 10;      // サブ目盛の刻む個数

    // Draw Meter Frame Parameter
    constexpr static int meter_frame_tick = 20;
    constexpr static int meter_frame_offx = 40;
    constexpr static int meter_frame_offy = 15;
    constexpr static int meter_frame_seg = 10;

    // Draw Text Border
    constexpr static int frame_a = 5;
    constexpr static int frame_half_width = 20;
    constexpr static int frame_heiht = 8;

    QLabel *value_label_;
    float normalized_angle;

    void curve(float pivot_x, float pivot_y, int& x, int& y, float r = 1280.0f * 2);
    void paintEvent(QPaintEvent*);

public:
    HUDCompass(QWidget *parent);
    void setValue(float);
};