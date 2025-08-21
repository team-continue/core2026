#pragma once

#include "gui_qt/IWidgetComponet.hpp"
#include "gui_qt/battle_param.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QWidget>
#include <QTimer>
#include <vector>

class HUDManager : public QObject{
    Q_OBJECT
    
    QTimer *update_timer_;

    std::vector<QWidget*> widget_components_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

    rclcpp::Node::SharedPtr node_;

public:

    HUDManager();
    ~HUDManager();
    void startUpdate();
    void stopUpdate();
    void addComponent(QWidget *component);
    void setNode(rclcpp::Node::SharedPtr node);
    bool setBattleParameter(BattleParamData &battle_param);
    rclcpp::Node::SharedPtr getNode();

private Q_SLOTS:
    void onUpdate();
};