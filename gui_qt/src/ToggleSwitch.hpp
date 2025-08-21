#pragma once

#include "CommonDefinition.hpp"
#include <QWidget>
#include <QPainter>
#include <QMouseEvent>

class ToggleSwitch : public QWidget {
    Q_OBJECT
public:
    ToggleSwitch(QWidget *parent = nullptr);
    bool isChecked();
    void setOffColor(QColor);
    void setOnColor(QColor);
    void doToggle();
    void setToggle(bool);
signals:
    void toggled(bool checked);

protected:
    QColor checked_color = QColor(76, 175, 80);
    QColor notchecked_color = QColor(100, 100, 100);

    void paintEvent(QPaintEvent *) override;
    void mousePressEvent(QMouseEvent *);

private:
    bool checked;
};