#include "HUDCamera.hpp"

EnemyMarker::EnemyMarker(QWidget *parent) 
:   QWidget(parent)
{
    setFixedSize(1280, 720);
}

void EnemyMarker::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    //const int view_offset_x = this->width() / 2;
    //const int view_offset_y = this->height() / 2;

    QPainter painter(this);
    QPen pen;
    painter.setRenderHint(QPainter::Antialiasing);
    
    const int size = 20;
    const int size_sub = 10;
    
    for (auto pose: enemy_poses_) {
        int enemy_x = pose.position.x;
        int enemy_y = pose.position.y;

        pen.setWidth(2);
        if (pose.position.z != 0) {
            pen.setColor(solid_red);
            painter.setPen(pen);
        } else {
            pen.setColor(solid);
            painter.setPen(pen);
        }
        painter.drawLine(enemy_x - size / 2, enemy_y, enemy_x + size / 2, enemy_y);
        painter.drawLine(enemy_x, enemy_y - size / 2, enemy_x, enemy_y + size / 2);
        painter.drawEllipse(enemy_x - size_sub / 2, enemy_y - size_sub / 2, size_sub, size_sub);
    }
}

void EnemyMarker::setMarker(std::vector<geometry_msgs::msg::Pose> poses) {
    enemy_poses_ = poses;
}

HUDCamera::HUDCamera(QWidget *parent)
:   QWidget(parent)
{
    label_ = new QLabel("", this);
    enemy_marker_ = new EnemyMarker(this);
    
    setFixedSize(1280, 720);
    
    label_->setFixedSize(1280, 720);
    label_->setStyleSheet(
        "\
        background-color: rgb(40, 40, 40);\
        color: rgb(255,255,255);\
        "
    );
    label_->setAlignment(Qt::AlignCenter);
    label_->setFont(QFont(USE_FONT, 10));
    label_->setText("No signal");
}

void HUDCamera::setEnemyPoses(std::vector<geometry_msgs::msg::Pose> poseArray) {
    enemy_marker_->setMarker(poseArray);
}

void HUDCamera::setImage(sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    QByteArray img = QByteArray(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
    auto qimage = QImage::fromData(img);
    auto imageMain = QPixmap::fromImage(qimage);
    label_->setPixmap(imageMain);
}