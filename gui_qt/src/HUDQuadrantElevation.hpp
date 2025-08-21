#pragma once

#include "CommonDefinition.hpp"
#include "gui_qt/IWidgetComponet.hpp"
#include <QWidget>
#include <QLabel>
#include <QtGui>
#include <sstream>
#include <iomanip>

class HUDQuadrantElevation : public QWidget, public IWidgetComponet {
    // parameter
    constexpr static int unit_scale = 5;            // でかい目盛の単位
    constexpr static int draw_scale = 10;
    constexpr static int scale_size = 10;             // 表示する目盛の長さ [px]
    constexpr static int scale_sub_size = 4;         // 表示するサブ目盛の長さ [px]
    constexpr static int scale_amount = 9;          // 目盛の刻む個数
    constexpr static int scale_sub_amount = 10;      // サブ目盛の刻む個数
    constexpr static float piv_x = 0;
    constexpr static float piv_y = 0;
    
    std::shared_ptr<QLabel> label_;
    float normalized_angle;
    
    void paintEvent(QPaintEvent*);

public:
    HUDQuadrantElevation(QWidget *parent);
    void setValue(float);
};