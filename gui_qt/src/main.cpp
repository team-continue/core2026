// 2025だぞ
// #include <QApplication>
// #include "MainGUIWidget.hpp"

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   QApplication app(argc, argv);
//   auto main_gui = new MainGUIWidget();

//   main_gui->show();
//   return app.exec();
// }

#include "HUDManager.hpp"
#include "HUDNode.hpp"
#include <QApplication>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    
    std::vector<std::string> args(argv, argv + argc);
    // 文字列の末尾が global_battle.param.yaml を検索する
    std::string param_path;
    for (int i = 0; i < argc; i++) {
        if (args[i] == "--params-file") {
            std::string target = "global_battle.param.yaml";
            auto path = args[i + 1];
            auto pos = path.rfind(target) + target.size() - path.size();
            //RCLCPP_INFO(rclcpp::get_logger("test"), "search : %s, pos = %d", path.c_str(), pos);
            if (pos == 0) {
                param_path = args[i+1];
            }
        }
    }

    auto manager = std::make_shared<HUDManager>();
    auto core = std::make_shared<HUDCore>(manager);
    auto node = std::make_shared<HUDNode>(core, param_path, rclcpp::NodeOptions());
    manager->setNode(node);

    //main_gui->show();
    return app.exec();
}