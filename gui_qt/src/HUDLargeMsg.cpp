#include "HUDLargeMsg.hpp"

HUDLargeMsg::HUDLargeMsg(QWidget *parent)
 :  QWidget(parent),
    start_point()
 {
    label_ = new QLabel("", this);
    setFixedSize(1280, 720);

    label_->setFixedWidth(width());
    label_->setAlignment(Qt::AlignCenter);
    label_->setMargin(margin / 2);
    label_->setFont(QFont(USE_FONT, 10));
    /*label_->setStyleSheet(
        "\
        color: rgb(81, 247, 205);\
        background: rgba(0,0,0,0)\
        "
    );*/

    auto palette = label_->palette();
    palette.setColor(QPalette::WindowText, solid_line);
    palette.setColor(QPalette::Window, QColor(0, 0, 0, 0));
    label_->setPalette(palette);
}

void HUDLargeMsg::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    
    auto now_point = std::chrono:: system_clock::now();
    auto duration = now_point - start_point;
    float duration_sec = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000.0f;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    if (duration.count() > 0 && is_animation) {
        if (duration_sec < open_time) {
            progress = duration_sec / open_time;
        } else if (duration_sec < open_time + duration_time) {
            progress = 1.0f;
        } else {
            progress = 1.0f - (duration_sec - (open_time + duration_time)) / close_time;
            progress = progress < 0 ? 0 : progress;
        }
    }

    
    
    
    // Draw Background
    int cx = label_->width() / 2;
    int cy = msg_height / 2;
    int ref_height = msg_height * progress;
    int upper_y = cy - ref_height / 2;
    int lower_y = cy + ref_height / 2;
    int ref_upper_y = upper_y + margin / 4;
    int ref_lower_y = lower_y- margin / 4;
    
    painter.setOpacity(progress);
    
    painter.setBrush(background);
    painter.drawRect(0, upper_y, 1280, ref_height);
    
    painter.setPen(solid_line);
    painter.drawLine(cx - ref_upper_x, ref_upper_y, 0, ref_upper_y);
    painter.drawLine(cx + ref_upper_x, ref_upper_y, 1280, ref_upper_y);
    painter.drawLine(cx - ref_lower_x, ref_lower_y, 0, ref_lower_y);
    painter.drawLine(cx + ref_lower_x, ref_lower_y, 1280, ref_lower_y);
    
    //painter.drawLine();
    //painter.drawLine();
    //painter.drawLine();
    //painter.drawLine();
    
    
    
    // Draw Text
    auto blink_switch = blink_pattern[blink_index++];
    blink_index = blink_index % 40;
    if (blink_switch) {
        if (blink_state) {
            painter.setOpacity(progress);
        } else {
            painter.setOpacity(0);
        }

        // 意図してる実装ではなかったが、いい感じに動いてるのでヨシッ！
        //blink_state = !blink_state;
        blink_state = false;
        
    } else {
        painter.setOpacity(progress);
    }
    int label_w = rendered_label.width();
    int label_h = rendered_label.height();
    float label_scale = 1.0f + 0.2f * (1.0f - progress);
    int label_offset_x = label_w * 0.2f * (1.0f - progress) / 2.f;
    int label_offset_y = label_h * 0.2f * (1.0f - progress) / 2.f;
    QRect label_rect(-label_offset_x, -label_offset_y, label_w * label_scale, label_h * label_scale);
    painter.drawImage(label_rect, rendered_label);

}

void HUDLargeMsg::setMsg(std::string msg, float duration, QColor color) {
    label_->setText(QString::fromStdString(msg));
    label_->adjustSize();
    msg_height = label_->height();

    rendered_label = QImage(label_->size(), QImage::Format_ARGB32);
    rendered_label.fill(Qt::transparent);
    QPainter painter(&rendered_label);

    QPalette palette;

    /*label_->setStyleSheet(
        "\
        color: rgb(81, 247, 205);\
        background: rgba(0,0,0,0)\
        "
    );*/
    palette = label_->palette();
    palette.setColor(QPalette::WindowText, color);
    label_->setPalette(palette);

    label_->render(&painter);
    
    /*label_->setStyleSheet(
        "\
        color: rgba(0, 0, 0, 0);\
        background: rgba(0,0,0,0)\
        "
    );*/
    palette = label_->palette();
    palette.setColor(QPalette::WindowText, QColor(0,0,0,0));
    label_->setPalette(palette);
    
    
    move(0, parentWidget()->height() / 2 - msg_height / 2);
    is_animation = true;
    duration_time = duration;
    blink_index = 0;
    blink_state = 0;
    start_point = std::chrono:: system_clock::now();
}