#pragma once

#include <QWidget>

class HUDDebugOverlay : public QWidget {
public:
    explicit HUDDebugOverlay(QWidget *parent = nullptr);

protected:
    void paintEvent(QPaintEvent *event) override;
};
