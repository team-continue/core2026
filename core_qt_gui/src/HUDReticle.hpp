#pragma once 

#include "CommonDefinition.hpp"
#include "gui_qt/IWidgetComponet.hpp"
#include <QWidget>
#include <QtGui>

class HUDReticle : public QWidget, public IWidgetComponet {
    constexpr static QColor solid_ = solid;
    constexpr static QColor solid_red_ = solid_red;
    constexpr static QColor translucent_ = translucent;
    constexpr static QColor translucent_black_ = translucent_black;
    constexpr static int circle_scale = 200;
    constexpr static int circle_scale_sub = 208;
    constexpr static int crosshair_space = 20;
    constexpr static int crosshair_length = 5;
    constexpr static int ammo1_size = 220;
    constexpr static int ammo1_width = 6;    

    int offset_x = 0;
    int offset_y = 0;
    int ammo1_max = 10;
    float ammo1_progress = 0.0f;

    void paintEvent(QPaintEvent*);
public:
    HUDReticle(QWidget *parent);
    void setPosition(int x, int y);
    void setMaxAmmo(int v);
    void setAmmo(int v);
};