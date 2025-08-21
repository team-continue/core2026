#include "HUDHazard.hpp"


HUDHazard::HUDHazard(QWidget *parent)
:   QWidget(parent)
{
    state_ = false;
    label_ = new QLabel("", this);
    
    setFixedSize(1280, 720);
    
    auto palette = label_->palette();
    palette.setColor(QPalette::WindowText, solid_red);
    label_->setPalette(palette);
    label_->setAlignment(Qt::AlignHCenter);
    label_->setFont(QFont(USE_FONT, 20));
    label_->setContentsMargins(10,5,10,5);
    label_->setText("EMERGENCY");
    

    label_->adjustSize();
    label_->move(640 - label_->width() / 2, 150);
    
    label_->setVisible(false);
    
    update();
}

void HUDHazard::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    //const int view_offset_x = this->width() / 2;
    //const int view_offset_y = this->height() / 2;

    QPainter painter(this);
    QPen pen;
    painter.setRenderHint(QPainter::Antialiasing);
    
    if (state_) {
        label_->setVisible(true);
        
        QRect  outline(label_->x(), label_->y(), label_->width(), label_->height());
        
        pen.setColor(solid_red);
        pen.setWidth(5);
        painter.setPen(pen);
        painter.drawRect(outline);
    } else {
        label_->setVisible(false);
    }
}

void HUDHazard::setState(bool state) {
    state_ = state;
    this->update();
}