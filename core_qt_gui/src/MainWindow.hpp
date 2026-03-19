#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QWidget>
#include <QKeyEvent>
#include <QFontDatabase>

class MainWindow : public QWidget {
    Q_OBJECT
    
    int font_id;
public:
    MainWindow();
    ~MainWindow();
    void keyPressEvent(QKeyEvent *event);
};