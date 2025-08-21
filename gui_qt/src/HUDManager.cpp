#include "HUDManager.hpp"
#include "HUDNode.hpp"

HUDManager::HUDManager()
 :  widget_components_(),
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
    update_timer_ = new QTimer;
    connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));
    startUpdate();
}

HUDManager::~HUDManager() {
    delete update_timer_;
}

void HUDManager::startUpdate() {
    float Hz = 30.0f;
    float interval = 1000.0f / Hz;
    update_timer_->start(interval);
}

void HUDManager::stopUpdate() {
    update_timer_->stop();
}

void HUDManager::onUpdate() {
    auto start = std::chrono::high_resolution_clock::now();
    executor_->spin_some();

    for (auto e : widget_components_) {
        e->update();
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(getNode()->get_logger(), "onUpdate:= %lf", elapsed.count());
}

void HUDManager::addComponent(QWidget *component) {
    widget_components_.emplace_back(component);
}

void HUDManager::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    executor_->add_node(node_);
}

rclcpp::Node::SharedPtr HUDManager::getNode() {
    return node_;
}

bool HUDManager::setBattleParameter(BattleParamData &battle_param) {
    auto node = std::dynamic_pointer_cast<HUDNode>(node_);
    if (node == nullptr) return false;
    return node->requestSetBattleParameter(battle_param);
}