#include "HUDReticle.hpp"

HUDReticle::HUDReticle(QWidget *parent)
 :  QWidget(parent)
{
    setFixedSize(1280, 720);
}
    
void HUDReticle::setPosition(int x, int y) {
    offset_x = x;
    offset_y = y;
}

void HUDReticle::setMaxAmmo(int v) {
    ammo1_max = v < 1 ? 1 : v;
}

void HUDReticle::setAmmo(int v) {
    ammo1_progress = (float)v / ammo1_max;
}

void HUDReticle::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    int cx = parentWidget()->width() / 2;
    int cy = parentWidget()->height() / 2;
    
    

    QRadialGradient gradient(cx, cy, circle_scale);
    gradient.setColorAt(0.35, QColor(255, 255, 255, 0));    // inner color
    gradient.setColorAt(1, QColor(255, 255, 255, 20));      // outer color

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    QPen pen;
    pen.setWidth(1);
    pen.setColor(translucent_);
    painter.setPen(pen);
    


    // Draw circle
    painter.setBrush(gradient);
    pen.setWidth(1); painter.setPen(pen);
    painter.drawEllipse(cx - circle_scale / 2, cy - circle_scale / 2, circle_scale, circle_scale);

    

    // Draw tri line
    QLine c_line_1(cx, cy - circle_scale / 2 + 2, cx, cy - circle_scale / 2 + 10);
    QTransform transform;
    
    transform.translate(cx, cy);
    transform.rotate(120);
    transform.translate(-cx, -cy);
    QLine c_line_2 = transform.map(c_line_1);
    
    transform.translate(cx, cy);
    transform.rotate(120);
    transform.translate(-cx, -cy);
    QLine c_line_3 = transform.map(c_line_1);

    painter.drawLine(c_line_1);
    painter.drawLine(c_line_2);
    painter.drawLine(c_line_3);

    pen.setWidth(2); painter.setPen(pen);
    painter.drawEllipse(cx - circle_scale_sub / 2, cy - circle_scale_sub / 2, circle_scale_sub, circle_scale_sub);
    


    // Draw CrossHair
    pen.setColor(solid_); pen.setWidth(4); painter.setPen(pen);
    painter.drawLine(cx + crosshair_space, cy, cx + crosshair_space + crosshair_length, cy);
    painter.drawLine(cx - crosshair_space, cy, cx - crosshair_space - crosshair_length, cy);
    painter.drawLine(cx, cy + crosshair_space, cx, cy + crosshair_space + crosshair_length);
    painter.drawLine(cx, cy - crosshair_space, cx, cy - crosshair_space - crosshair_length);



    // Draw Ammo bar Frame
    QLine ammo1_startline(cx - ammo1_size / 2, cy, cx - ammo1_size / 2 - ammo1_width, cy);
    QTransform transform_ammo1;
    transform_ammo1.translate(cx, cy);
    transform_ammo1.rotate(-60);
    transform_ammo1.translate(-cx, -cy);
    QLine ammo1_endline = transform_ammo1.map(ammo1_startline);

    pen.setColor(solid_); pen.setWidth(1); painter.setPen(pen);
    painter.drawLine(ammo1_startline);
    painter.drawLine(ammo1_endline);
    painter.drawArc(cx - ammo1_size / 2, cy - ammo1_size / 2, ammo1_size, ammo1_size, 180 * 16, 60 * 16);

    // Draw Ammo bar
    const float bar_start = -(120 + 2) * 16;
    const float max_span = -(60 - 4) * 16;
    
    float ammo1_bar_span = max_span * ammo1_progress;
    int ammo1_adj_size = ammo1_size + ammo1_width * 1.5f;
    
    // background
    //pen.setColor(translucent_black_); pen.setWidth(ammo1_width / 2); painter.setPen(pen);
    //painter.drawArc(cx - ammo1_adj_size / 2, cy - ammo1_adj_size / 2, ammo1_adj_size, ammo1_adj_size, bar_start, max_span);
    
    // foreground
    if (ammo1_progress == 0) {
        pen.setColor(solid_red_); pen.setWidth(ammo1_width / 2); painter.setPen(pen);
        painter.drawArc(cx - ammo1_adj_size / 2, cy - ammo1_adj_size / 2, ammo1_adj_size, ammo1_adj_size, bar_start, max_span);
    } else {
        pen.setColor(solid_); pen.setWidth(ammo1_width / 2); painter.setPen(pen);
        painter.drawArc(cx - ammo1_adj_size / 2, cy - ammo1_adj_size / 2, ammo1_adj_size, ammo1_adj_size, bar_start, ammo1_bar_span);
    }
}
