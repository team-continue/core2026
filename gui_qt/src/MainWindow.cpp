#include "MainWindow.hpp"

#include "SettingMenu.hpp"
#include <rclcpp/rclcpp.hpp>

MainWindow::MainWindow()
 :  QWidget(nullptr)
{
    std::string shared_path = ament_index_cpp::get_package_share_directory("gui_qt");
    std::string font_path = shared_path + "/font/Michroma-Regular.ttf";
    font_id = QFontDatabase::addApplicationFont(font_path.c_str());
    //QString family = QFontDatabase::applicationFontFamilies(id).at(0);
    setFixedSize(1280, 720);
};

MainWindow::~MainWindow() {
    rclcpp::shutdown();
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Space) {
        if (this->isFullScreen()) {
            showNormal();
            RCLCPP_INFO(rclcpp::get_logger("test"), "Key press. Unset Fullscreen.");
        } else {
            showFullScreen();
            RCLCPP_INFO(rclcpp::get_logger("test"), "Key press. Set Fullscreen.");
        }
    } else if (event->key() == Qt::Key_2) {
        auto widget = dynamic_cast<SettingMenu*>(findChild<QWidget*>("SettingMenu"));
        if (!widget->isHidden()) {
            widget->inputCursorNext();
        }
    } else if (event->key() == Qt::Key_8) {
        auto widget = dynamic_cast<SettingMenu*>(findChild<QWidget*>("SettingMenu"));
        if (!widget->isHidden()) {
            widget->inputCursorPrev();
        }
    } else if (event->key() == Qt::Key_4) {
        auto widget = dynamic_cast<SettingMenu*>(findChild<QWidget*>("SettingMenu"));
        if (!widget->isHidden()) {
            widget->inputValueDown();
        }
    } else if (event->key() == Qt::Key_6) {
        auto widget = dynamic_cast<SettingMenu*>(findChild<QWidget*>("SettingMenu"));
        if (!widget->isHidden()) {
            widget->inputValueUp();
        }
    } else if (event->key() == Qt::Key_Escape) {
        auto widget = dynamic_cast<SettingMenu*>(findChild<QWidget*>("SettingMenu"));
        if (widget->isHidden()) {
            widget->open();
        } else {
            widget->inputCursorBack();
        }
    } else if (event->key() == Qt::Key_Enter) {
        auto widget = dynamic_cast<SettingMenu*>(findChild<QWidget*>("SettingMenu"));
        if (!widget->isHidden()) {
            widget->inputCursorOK();
        }
    } else {
        QWidget::keyPressEvent(event);
    }
}
