#include "HUDAmmoPanel.hpp"


HUDAmmoPanel::HUDAmmoPanel(QWidget *parent) 
  : QWidget(parent)
{
    value_label_ = new QLabel("", this);
    label_ = new QLabel("", this);
    layout_ = new QHBoxLayout(this);

    QPalette palette;
    move(1000, 600);

    value_label_->setText("RELOAD");
    palette = value_label_->palette();
    palette.setColor(QPalette::WindowText, solid_);
    value_label_->setPalette(palette);
    value_label_->setAlignment(Qt::AlignRight | Qt::AlignBottom);
    value_label_->setFont(QFont(USE_FONT, 10));

    label_->setText("U.Disk");
    palette = label_->palette();
    palette.setColor(QPalette::WindowText, solid_);
    label_->setPalette(palette);
    label_->setAlignment(Qt::AlignLeft | Qt::AlignBottom);
    label_->setFont(QFont(USE_FONT, 10));

    layout_->addWidget(label_);
    layout_->addSpacing(30);
    layout_->addWidget(value_label_);

    setLayout(layout_);
}

void HUDAmmoPanel::setValue(int v) {
    
    if (v <= 0) {
        value_label_->setText("RELOAD");
        
        // 警告色に設定
        auto palette = value_label_->palette();
        palette.setColor(QPalette::WindowText, solid_red_);
        value_label_->setPalette(palette); 
    }else {
        std::stringstream ss;
        ss << v;
        value_label_->setText(QString::fromStdString(ss.str()));
        
        // 通常色に設定
        auto palette = value_label_->palette();
        palette.setColor(QPalette::WindowText, solid_);
        value_label_->setPalette(palette);
    }
}

void HUDAmmoPanel::setReload() {
}