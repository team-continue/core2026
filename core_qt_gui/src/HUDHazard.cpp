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
    info_->setStyleSheet(
    "background-color: rgba(0, 0, 0, 160);"
    "padding: 8px;"
);

    label_->adjustSize();
    label_->move(640 - label_->width() / 2, 150);

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
         // Background first so all HUD drawings stay on top.
        painter.setPen(QPen(QColor(0, 0, 0, 0)));
        painter.setBrush(QColor(0, 0, 0, 160));
        painter.drawRect(QRect(0, 0, label_->width(), label_->height()));
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

void HUDHazard::setInfo(std::string info) {
    size_t pos = 0;
    while ((pos = info.find(", ", pos)) != std::string::npos) {
        info.replace(pos, 2, "\n");
    }
    setInfoText(info);
    this->update();
}

void HUDHazard::setInfoText(std::string text) {
    info_->setText(text.c_str());
    info_->adjustSize();
    info_->move(640 - info_->width() / 2, 480);
}