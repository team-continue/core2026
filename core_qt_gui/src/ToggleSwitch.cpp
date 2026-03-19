#include "ToggleSwitch.hpp"

ToggleSwitch::ToggleSwitch(QWidget *parent)
 :  QWidget(parent), checked(false)
{
    setFixedSize(50, 20);
}

bool ToggleSwitch::isChecked() {
    return checked;
}

void ToggleSwitch::setOffColor(QColor color) {
    notchecked_color = color;
}

void ToggleSwitch::setOnColor(QColor color) {
    checked_color = color;
}

void ToggleSwitch::doToggle() {
    checked = !checked;
    emit toggled(checked);
    update();  // 再描画
}

void ToggleSwitch::setToggle(bool b) {
    checked = b;
    emit toggled(checked);
    update();  // 再描画
}

void ToggleSwitch::paintEvent(QPaintEvent *) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 背景（スイッチの台座）
    painter.setBrush(checked ? checked_color : notchecked_color);
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(0, 0, width(), height(), 12, 12);

    // スイッチ部分（丸いノブ）
    painter.setBrush(Qt::white);
    int knobX = checked ? width() - height() : 0;
    painter.drawEllipse(knobX, 0, height(), height());
}

void ToggleSwitch::mousePressEvent(QMouseEvent *) {
    doToggle();
}
