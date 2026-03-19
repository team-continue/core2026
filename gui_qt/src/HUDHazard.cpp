#include "HUDHazard.hpp"


HUDHazard::HUDHazard(QWidget *parent)
:   QWidget(parent)
{
    state_ = false;
    label_ = new QLabel("", this);
    info_ = new QLabel("", this);
    
    setFixedSize(1280, 720);
    
    auto palette = label_->palette();
    palette.setColor(QPalette::WindowText, solid_red);
    label_->setPalette(palette);
    label_->setAlignment(Qt::AlignHCenter);
    label_->setFont(QFont(USE_FONT, 30));
    label_->setContentsMargins(10,5,10,5);
    label_->setText("EMERGENCY");

    palette = info_->palette();
    palette.setColor(QPalette::WindowText, solid_red);
    info_->setPalette(palette);
    info_->setAlignment(Qt::AlignHCenter);
    info_->setFont(QFont(USE_FONT, 20));
    info_->setContentsMargins(10,5,10,5);
    info_->setText("");

    setInfoText("");
    
    info_->setVisible(false);
    
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
        info_->setVisible(true);
        
        QRect  outline(label_->x(), label_->y(), label_->width(), label_->height());
        
        pen.setColor(solid_red);
        pen.setWidth(5);
        painter.setPen(pen);
        painter.drawRect(outline);
    } else {
        label_->setVisible(false);
        info_->setVisible(false);
    }
}

void HUDHazard::setState(bool state) {
    state_ = state;
    this->update();
}

void HUDHazard::setInfo(int8_t info) {
    // 数字が大きい方がつよい
    
    // 45 ぶつりきんてい
    if (info == 45) {
        setInfoText("Reason: HARD Emergency");
    }
    
    // 34 ソフトキンテイ
    if (info == 34) {
        setInfoText("Reason: SOFT Emergency");
    }

    // 23 teensy timeout
    if (info == 23) {
        setInfoText("Reason: Teensy timeout");
    }
    
    // 12 receiver timeout
    if (info == 12) {
        setInfoText("Reason: Receiver timeout");
    }

    // 11 destory
    if (info == 11) {
        setInfoText("Reason: Destoryed");
    }

    // 00 非緊急
    if (info == 0) {
        setInfoText("");
        return;
    }

    this->update();

}

void HUDHazard::setInfoText(std::string text) {
    info_->setText(text.c_str());
    info_->adjustSize();
    info_->move(640 - info_->width() / 2, 440 + info_->height());
}