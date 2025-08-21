#include "HUDSpeed.hpp"

HUDSpeed::HUDSpeed(QWidget *parent)
:   QWidget(parent),
    label_(std::make_shared<QLabel>("", this)),
    normalized_speed(0)
{
    
    label_->setFixedSize(80, 300);
    label_->move(-40, 0);
    label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    label_->setFont(QFont(USE_FONT, 6));
    auto palette = label_->palette();
    palette.setColor(QPalette::WindowText, solid);
    label_->setPalette(palette);
    label_->setStyleSheet(
        "\
        background-color: rgba(0,0,0,0);\
        "
    );
    
    setFixedSize(80,300);
    move(0, this->parentWidget()->height() / 2 - size().height()/2);
    setStyleSheet(
        "\
        background-color: rgba(0,0,0,0);\
        "
    );

    setValue(0);
}

void HUDSpeed::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    
    static int view_offset_x = this->width() / 2 + 30;
    static int view_offset_y = this->height() / 2;
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    auto solid_color = solid;
    solid_color.setAlpha(100);
    painter.setPen(solid_color);
    
    float value = normalized_speed;
    
    // draw scale
    for (int i = -5; i < scale_amount + 5; i++) {
        int px = 0;
        int py = draw_scale * (scale_sub_amount + 1) * -i - draw_scale * (scale_sub_amount + 1) * (float)-value / unit_scale;
        
        int x1 = px;
        int y1 = py;
        int x2 = px - scale_size;
        int y2 = py;
        
        painter.drawLine(x1 + view_offset_x, y1 + view_offset_y, x2 + view_offset_x, y2 + view_offset_y);
        
        // draw sub scale
        for (int j = 0; j < scale_sub_amount; j++) {
            int qx = 0;
            int qy = py - draw_scale * (j + 1);
            
            int x3 = qx;
            int y3 = qy;
            int x4 = qx - scale_sub_size;
            int y4 = qy;
            
            float alpha = abs(y3) < label_->size().height() / 2 - 5? 100 : 0;

            solid_color.setAlpha(alpha);
            painter.setPen(solid_color);
            
            painter.drawLine(x3 + view_offset_x, y3 + view_offset_y, x4 + view_offset_x, y4 + view_offset_y);
        }
    }

    // draw outline
    auto ol_ox = view_offset_x - 15;
    auto ol_oy = view_offset_y;
    painter.setPen(solid);
    painter.drawLine(0 + ol_ox,0 + ol_oy,-8 + ol_ox,8 + ol_oy);
    painter.drawLine(-8 + ol_ox,8 + ol_oy,-40 + ol_ox,8 + ol_oy);
    painter.drawLine(-40 + ol_ox,8 + ol_oy,-40 + ol_ox,-8 + ol_oy);
    painter.drawLine(-40 + ol_ox,-8 + ol_oy,-8 + ol_ox,-8 + ol_oy);
    painter.drawLine(-8 + ol_ox,-8 + ol_oy,0 + ol_ox,0 + ol_oy);

    // draw framwe
    painter.setPen(solid);

    auto fr_sx_upper = view_offset_x - 45;
    auto fr_sy_upper = 0;
    painter.drawLine(fr_sx_upper, fr_sy_upper, fr_sx_upper + 50, fr_sy_upper);
    painter.drawLine(fr_sx_upper + 50, fr_sy_upper, fr_sx_upper + 50, fr_sy_upper + 5);
    
    auto fr_sx_lower = view_offset_x - 45;
    auto fr_sy_lower = label_->size().height() - 1;
    painter.drawLine(fr_sx_lower, fr_sy_lower, fr_sx_lower + 50, fr_sy_lower);
    painter.drawLine(fr_sx_lower + 50, fr_sy_lower, fr_sx_lower + 50, fr_sy_lower - 5);
}

void HUDSpeed::setValue(float v) {
    std::stringstream ss;
    normalized_speed = v;
    ss << std::setw(3) << (int)normalized_speed;
    
    label_->setText(QString::fromStdString(ss.str()));
}