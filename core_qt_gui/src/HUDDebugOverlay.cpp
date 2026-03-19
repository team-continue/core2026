#include "gui_qt/HUDDebugOverlay.hpp"
#include <QPainter>
#include <QPen>

HUDDebugOverlay::HUDDebugOverlay(QWidget *parent)
    : QWidget(parent) {
    setFixedSize(1280, 720);
    move(0, 0);
    setAttribute(Qt::WA_TransparentForMouseEvents, true);
}

void HUDDebugOverlay::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setPen(QPen(QColor(0, 0, 0, 0)));
    painter.setBrush(QColor(30, 120, 255, 100)); // semi-transparent blue

    // Top band with center cutout (based on the provided diagram)
    // Left block: 270x270 at top-left
    painter.drawRect(QRect(0, 0, 270, 270));
    // Right block: 270x270 at top-right
    painter.drawRect(QRect(1280 - 270, 0, 270, 270));
    // Top center strip: width 740, height 100
    painter.drawRect(QRect(270, 0, 740, 100));

    // Bottom-left block: 300x100
    painter.drawRect(QRect(0, 720 - 100, 300, 100));
}
