#pragma once

#include "gui_qt/battle_param.hpp"
#include "HUDManager.hpp"
#include "ToggleSwitch.hpp"
#include "rclcpp/rclcpp.hpp"
#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QSlider>
#include <vector>
#include <utility>


class SettingMenu : public QWidget {
    Q_OBJECT

    void paintEvent(QPaintEvent *) override;
    bool isSelectedSlider() { if (cursor_pos < SettingItem::Team) return true; return false; }

    std::shared_ptr<HUDManager> manager_;

    QGridLayout *layout_;
    
    QLabel *label_title_;

    QLabel *label_aim_sensitivity_;
    QSlider *slider_aim_sensitivity_;
    QLabel *value_aim_sensitivity_;

    QLabel *label_max_velocity_;
    QSlider *slider_max_velocity_;
    QLabel *value_max_velocity_;

    QLabel *label_max_omega_;
    QSlider *slider_max_omega_;
    QLabel *value_max_omega_;

    QLabel *label_max_shooter_;
    QSlider *slider_max_shooter_;
    QLabel *value_max_shooter_;

    QLabel *label_team_;
    ToggleSwitch *switch_team_;
    QLabel *value_team_;

    QLabel *label_auto_aim_;
    ToggleSwitch *switch_auto_aim_;
    QLabel *value_auto_aim_;

    std::vector<std::tuple<QLabel*, QObject*, QLabel*>> items;

    enum SettingItem {
        AimSensitivity,
        MaxVelocity,
        MaxOmega,
        MaxShooterSpeed,
        Team,
        AutoAim
    } SettingItem;

    int cursor_pos = 0;
public:
    SettingMenu(std::shared_ptr<HUDManager>, QWidget *);
    void setParam(BattleParam &param);
    void open();
    void inputCursorNext();
    void inputCursorPrev();
    void inputCursorOK();
    void inputCursorBack();
    void inputValueUp();
    void inputValueDown();
    void setRangeAimSensitivity(double min, double max);
    void setRangeMaxVelocity(double min, double max);
    void setRangeMaxOmega(double min, double max);
    void setRangeMaxShooter(double min, double max);
};