#include "HUDCompass.hpp"

HUDCompass::HUDCompass(QWidget *parent)
  : QWidget(parent),
  normalized_angle(0)
  {
    value_label_ = new QLabel("", this);

    move(340, 640);

    value_label_->move(0, 0);
    value_label_->setFixedSize(600, 80);
    value_label_->setStyleSheet(
        "\
        margin-top: 35px;\
        "
    );
    auto palette = value_label_->palette();
    palette.setColor(QPalette::WindowText, solid);
    value_label_->setPalette(palette);
    value_label_->setAlignment(Qt::AlignCenter);
    value_label_->setFont(QFont(USE_FONT, 6));
    setValue(0);
}

void HUDCompass::curve(float pivot_x, float pivot_y, int& x, int& y, float r) {
    float vx = x - pivot_x;
    float vy = y - pivot_y;
    float theta = x / r;
    
    x = vx * cosf(theta) - vy * -sinf(theta);
    y = vx * sin(theta) + vy * cos(theta);
}

void HUDCompass::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    
    const int view_offset_x = this->width() / 2;
    const int view_offset_y = this->height() / 2;
    float piv_x = 0;
    float piv_y = 0;
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    painter.setPen(solid);
    
    float value = normalized_angle;
    
    // draw scale
    for (int i = -5; i < scale_amount + 5; i++) {
        int px = draw_scale * (scale_sub_amount + 1) * i - draw_scale * (scale_sub_amount + 1) * (float)value / unit_scale;
        int py = -scale_size / 2.0;
        
        int x1 = px;
        int y1 = py;
        int x2 = px;
        int y2 = py + scale_size;
        
        curve(piv_x, piv_y, x1, y1);
        curve(piv_x, piv_y, x2, y2);
        
        painter.drawLine(x1 + view_offset_x, y1 + view_offset_y, x2 + view_offset_x, y2 + view_offset_y);
        
        // draw sub scale
        for (int j = 0; j < scale_sub_amount; j++) {
            int qx = px + draw_scale * (j + 1);
            int qy = -scale_sub_size / 2.0;
            
            int x3 = qx;
            int y3 = qy;
            int x4 = qx;
            int y4 = qy + scale_sub_size;
            
            curve(piv_x, piv_y, x3, y3);
            curve(piv_x, piv_y, x4, y4);
            
            float alpha = 255 - abs(x3) / 700.0f * 255 * 2;
            alpha = alpha > 255 ? 255 : alpha < 0 ? 0 : alpha;
            auto solid_alpha = solid;
            solid_alpha.setAlpha(alpha);
            painter.setPen(solid_alpha);
            
            painter.drawLine(x3 + view_offset_x, y3 + view_offset_y, x4 + view_offset_x, y4 + view_offset_y);
        }
    }

    int meter_frame_x = view_offset_x;
    int meter_frame_y = view_offset_y;
    auto meter_frame_color = solid;
    for (int i = 0; i < meter_frame_seg; i++) {
        meter_frame_color.setAlphaF(1.0f - (float)i / meter_frame_seg);
        painter.setPen(meter_frame_color);

        int px1 = meter_frame_offx + meter_frame_tick * i;
        int py1 = meter_frame_offy;
        int px2 = meter_frame_offx + meter_frame_tick * (i + 1);
        int py2 = meter_frame_offy;
        curve(piv_x, piv_y, px1, py1);
        curve(piv_x, piv_y, px2, py2);
        painter.drawLine(px1 + meter_frame_x, py1 + meter_frame_y, px2 + meter_frame_x, py2 + meter_frame_y);
        
        int px3 = -meter_frame_offx - meter_frame_tick * i;
        int py3 = meter_frame_offy;
        int px4 = -meter_frame_offx - meter_frame_tick * (i + 1);
        int py4 = meter_frame_offy;
        curve(piv_x, piv_y, px3, py3);
        curve(piv_x, piv_y, px4, py4);
        painter.drawLine(px3 + meter_frame_x, py3 + meter_frame_y, px4 + meter_frame_x, py4 + meter_frame_y);
    }

    // Draw frame
    int sx = view_offset_x;
    int sy = view_offset_y + 10;
    painter.setPen(solid);
    painter.drawLine(sx, sy, sx - frame_half_width, sy + frame_a);
    painter.drawLine(sx - frame_half_width, sy + frame_a, sx - frame_half_width, sy + frame_a + frame_heiht);
    painter.drawLine(sx - frame_half_width, sy + frame_a + frame_heiht, sx + frame_half_width, sy + frame_a + frame_heiht);
    painter.drawLine(sx + frame_half_width, sy + frame_a + frame_heiht, sx + frame_half_width, sy + frame_a);
    painter.drawLine(sx + frame_half_width, sy + frame_a, sx, sy);
}

void HUDCompass::setValue(float v) {
    std::stringstream ss;
    int n = v / 360;
    float angle = v - n * 360;
    
    if (angle < 0) {
        angle = 360 + angle;
    }
    normalized_angle = angle;
    ss << std::setw(3) << (int)normalized_angle;
    
    value_label_->setText(QString::fromStdString(ss.str()));
}