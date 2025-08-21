#pragma once

#include "CommonDefinition.hpp"
#include "MainWindow.hpp"
#include "HUDManager.hpp"
#include "HUDCamera.hpp"
#include "HUDCompass.hpp"
#include "HUDHPPanel.hpp"
#include "HUDAmmoPanel.hpp"
#include "HUDQuadrantElevation.hpp"
#include "HUDSpeed.hpp"
#include "HUDReticle.hpp"
#include "HUDLargeMsg.hpp"
#include "HUDHazard.hpp"
#include "SettingMenu.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>
#include <vector>
#include <memory>

class HUDCore {
    std::shared_ptr<HUDManager> manager_;
    MainWindow *main_window_;
    HUDCamera *camera1_;
    HUDCompass *compass_;
    HUDHPPanel *hp_panel_;
    HUDAmmoPanel *ammo_panel_;
    HUDQuadrantElevation *qe_;
    HUDSpeed *speed_;
    QLabel *speed_label_;
    QLabel *qe_label_;
    HUDReticle *reticle_;
    HUDLargeMsg *large_msg_;
    HUDHazard *hazard_;
    SettingMenu *setting_menu_;


    int camera_select = 0;

public:
    HUDCore(std::shared_ptr<HUDManager> manager)
    :   manager_(manager)
    {
        main_window_ = new MainWindow;
        camera1_ = new HUDCamera(main_window_);
        compass_ = new HUDCompass(main_window_);
        hp_panel_ = new HUDHPPanel(main_window_);
        ammo_panel_ = new HUDAmmoPanel(main_window_);
        qe_ = new HUDQuadrantElevation(main_window_);
        speed_ = new HUDSpeed(main_window_);
        speed_label_ = new QLabel("SPEED", main_window_);
        qe_label_ = new QLabel("Q.E.", main_window_);
        reticle_ = new HUDReticle(main_window_);
        large_msg_ = new HUDLargeMsg(main_window_);
        hazard_ = new HUDHazard(main_window_);
        setting_menu_ = new SettingMenu(manager_, main_window_);

        manager_->addComponent(camera1_);
        manager_->addComponent(compass_);
        manager_->addComponent(hp_panel_);
        manager_->addComponent(ammo_panel_);
        manager_->addComponent(qe_);
        manager_->addComponent(speed_);
        manager_->addComponent(reticle_);
        manager_->addComponent(large_msg_);

        QPalette palette;
        speed_label_->setAlignment(Qt::AlignBottom | Qt::AlignRight);
        speed_label_->setFont(QFont(USE_FONT, 6));
        palette = speed_label_->palette();
        palette.setColor(QPalette::WindowText, solid);
        speed_label_->setPalette(palette);
        speed_label_->adjustSize();
        speed_label_->move(speed_->x() + speed_->width() - speed_label_->width() - 10, speed_->y() - speed_label_->height());
        
        qe_label_->setAlignment(Qt::AlignBottom | Qt::AlignLeft);
        qe_label_->setFont(QFont(USE_FONT, 6));
        palette = qe_label_->palette();
        palette.setColor(QPalette::WindowText, solid);
        qe_label_->setPalette(palette);
        qe_label_->adjustSize();
        qe_label_->move(qe_->x() + 10, qe_->y() - qe_label_->height());

        //main_window_->setWindowFlags(Qt::Window);
        //main_window_->showFullScreen();
        main_window_->setWindowFlags(main_window_->windowFlags() | Qt::WindowStaysOnTopHint);
        main_window_->show();
        setting_menu_->hide();

        camera_select = 1;
    }
    
    void setMaxHP(int value) {
        hp_panel_->setMaxHP(value);
    }

    void setHP(int value) {
        RCLCPP_INFO(manager_->getNode()->get_logger(), "HP %d", value);
        hp_panel_->setHP(value);
        if (value == 0) {
            large_msg_->setMsg("<span style='font-size: 30pt;'>DESTROYED</span>", 3, QColor(0xFF, 0x40, 0x40));
        }
    }
    
    void setMaxAmmo(int value) {
        reticle_->setMaxAmmo(value);
    }

    void setAmmo(int value) {
        RCLCPP_INFO(manager_->getNode()->get_logger(), "Ammo %d", value);
        ammo_panel_->setValue(value);
        reticle_->setAmmo(value);
    }

    void setCompass(float value) {
        RCLCPP_INFO_THROTTLE(manager_->getNode()->get_logger(), *manager_->getNode()->get_clock(), 1000, "Compass %f", value);
        compass_->setValue(value);
    }

    void setSpeed(float value) {
        RCLCPP_INFO_THROTTLE(manager_->getNode()->get_logger(), *manager_->getNode()->get_clock(), 1000, "Speed %f", value);
        speed_->setValue(value);
    }
    
    void setQE(float value) {
        RCLCPP_INFO_THROTTLE(manager_->getNode()->get_logger(), *manager_->getNode()->get_clock(), 1000, "QE %f", value);
        qe_->setValue(value);
    }

    void setLog(std::string value) {
        RCLCPP_INFO(manager_->getNode()->get_logger(), "Msg: %s", value.c_str());
    }

    void setImage1(sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (camera_select == 1) {
            camera1_->setImage(msg);
        }
    }

    void setImage2(sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (camera_select == 2) {
            camera1_->setImage(msg);
        }
    }

    void setEnemyPoses(std::vector<geometry_msgs::msg::Pose> poseArray) {
        camera1_->setEnemyPoses(poseArray);
    }

    void setHazard(bool state) {
        RCLCPP_INFO(manager_->getNode()->get_logger(), "Set Hazard %d", state);
        hazard_->setState(state);
    }

    void onInputCamera1() {
        RCLCPP_INFO(manager_->getNode()->get_logger(), "Select Camera 1");
        camera_select = 1;
    }
    
    void onInputCamera2() {
        RCLCPP_INFO(manager_->getNode()->get_logger(), "Select Camera 2");
        camera_select = 2;
    }
    
    void onInputUp() {
        setting_menu_->inputCursorPrev();
    }
    
    void onInputDown() {
        setting_menu_->inputCursorNext();
    }
    
    void onInputLeft() {
        setting_menu_->inputValueDown();
    }
    
    void onInputRight() {
        setting_menu_->inputValueUp();
    }
    
    void onInputBack(BattleParam &param) {
        RCLCPP_INFO(manager_->getNode()->get_logger(), "Input Back");
        if (setting_menu_->isHidden()) {
            setting_menu_->setParam(param);
            setting_menu_->open();
        } else {
            setting_menu_->inputCursorBack();
        }
    }
    
    void onInputOK() {
        setting_menu_->inputCursorOK();
    }

    void setRangeAimSensitivity(double min, double max) {
        setting_menu_->setRangeAimSensitivity(min, max);
    }

    void setRangeMaxVelocity(double min, double max) {
        setting_menu_->setRangeMaxVelocity(min, max);
    }

    void setRangeMaxOmega(double min, double max) {
        setting_menu_->setRangeMaxOmega(min, max);
    }

    void setRangeMaxShooter(double min, double max) {
        setting_menu_->setRangeMaxShooter(min, max);
    }

    void setSettingParamater(BattleParam &param) {
        setting_menu_->setParam(param);
    }
};