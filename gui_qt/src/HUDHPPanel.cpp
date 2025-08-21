#include "HUDHPPanel.hpp"

HUDHPPanel::HUDHPPanel(QWidget *parent)
  : QWidget(parent),
  start_point_()
  {
    hp_value_label_ = new QLabel("", this);
    hp_diff_label_ = new QLabel("", this);
    hp_label_ = new QLabel("HP", this);
    hp_bar_ = new QProgressBar(this);
    sub_bar_1_ = new QProgressBar(this);
    sub_bar_2_ = new QProgressBar(this);
    layout_ = new QVBoxLayout(this);
    hp_layout_ = new QHBoxLayout();
    sub_layout_ = new QHBoxLayout();
    sub_bar_layout_ = new QVBoxLayout();

    setFixedSize(280, 150);
    move(70, 550);
    
    layout_->setAlignment(Qt::AlignCenter);
    layout_->addWidget(hp_diff_label_);
    layout_->addSpacing(-10);
    layout_->addLayout(hp_layout_);
    layout_->addWidget(hp_bar_);
    layout_->addLayout(sub_layout_);
    layout_->setSpacing(0);
    
    hp_diff_label_->setAlignment(Qt::AlignBottom | Qt::AlignRight);
    hp_diff_label_->setFont(QFont(USE_FONT, 10));
    hp_diff_label_->setText("-999");

    // 上段ラベルレイアウト
    hp_layout_->addWidget(hp_label_);
    hp_layout_->addWidget(hp_value_label_);

    hp_label_->setAlignment(Qt::AlignBottom | Qt::AlignLeft);
    hp_label_->setFont(QFont(USE_FONT, 10));
    auto palette = hp_label_->palette();
    palette.setColor(QPalette::WindowText, solid_);
    hp_label_->setPalette(palette);

    hp_value_label_->setAlignment(Qt::AlignRight);
    hp_value_label_->setFont(QFont(USE_FONT, 20));



    // 中段レイアウト
    hp_bar_->setTextVisible(false);
    hp_bar_->setFixedHeight(5);
    hp_bar_->setRange( 0, 100 );
    hp_bar_->setStyleSheet(
        "\
        background: rgba(255, 255, 255, 0);\
        border: 0px;\
        "
    );
    hp_bar_->setWindowOpacity(0);

    // 下段横レイアウト
    sub_layout_->addSpacing(100);
    sub_layout_->addLayout(sub_bar_layout_);

    // 下段縦レイアウト
    sub_bar_layout_->addSpacing(25);
    sub_bar_layout_-> addWidget(sub_bar_1_);
    sub_bar_layout_->addSpacing(20);
    sub_bar_layout_-> addWidget(sub_bar_2_);

    sub_bar_1_->setTextVisible(false);
    sub_bar_1_->setFixedHeight(5);
    sub_bar_1_->setRange( 0, 100 );
    sub_bar_1_->setStyleSheet(
        "\
        background: rgba(255,255,255,0);\
        border: 0px;\
        "
    );
    sub_bar_1_->setWindowOpacity(0);

    sub_bar_2_->setTextVisible(false);
    sub_bar_2_->setFixedHeight(5);
    sub_bar_2_->setRange( 0, 100 );
    sub_bar_2_->setStyleSheet(
        "\
        background: rgba(255,255,255,0);\
        border: 0px;\
        "
    );
    sub_bar_2_->setWindowOpacity(0);

    


    setMaxHP(100);
    setHP(100);
}

void HUDHPPanel::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    auto now_point = std::chrono:: system_clock::now();
    auto duration = now_point - start_point_;
    float duration_sec = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() / 1000.0f;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    constexpr static int margin_x = 5;
    constexpr static int margin_y = 1;
    const int px = hp_bar_->x() + margin_x;
    const int py = hp_bar_->y() + margin_y;
    
    // Draw HP Bar
    int bar_width = hp_bar_->width() - margin_x * 2;
    int bar_height = 4;
    
    QRect  hp_bar_back(px, py, bar_width, bar_height);
    painter.setBrush(translucent_black_);
    painter.setPen(QPen(QColor(0,0,0,0)));
    painter.drawRect(hp_bar_back);

    // Draw HP Bar Back
    painter.setPen(solid_);
    int frame_x = hp_bar_->x();
    int frame_y = hp_bar_->y();
    int frame_height = bar_height + margin_y;
    int frame_width = hp_bar_->width();
    painter.drawLine(frame_x, frame_y, frame_x, frame_y + frame_height);
    painter.drawLine(frame_x + frame_width, frame_y, frame_x + frame_width, frame_y + frame_height);

    

    // Draw Diff Label
    if (duration_sec <  diff_dura_time_) {
        // duration
        fade_progress_ = 1.0f;
    } else if (duration_sec - diff_dura_time_ < diff_fade_time_) {
        // fade out
        fade_progress_ =  1.0f - (duration_sec - diff_dura_time_) / diff_fade_time_;
    } else {
        // none
        fade_progress_ = 0;
    }
    auto palette = hp_diff_label_->palette();
    auto fade_red = solid_red_;
    fade_red.setAlpha(255 * fade_progress_);
    palette.setColor(QPalette::WindowText, fade_red);
    hp_diff_label_->setPalette(palette);

    
    
    // Draw HP Bar Front + Diff
    painter.setPen(QPen(QColor(0,0,0,0)));
    float hp_remain = (float)current_hp_ / max_hp_;
    float hp_remain_diff = (float)-diff_hp_ / max_hp_;

    painter.setBrush(solid_red_another_);
    QRect  hp_bar_diff(px, py, bar_width * (hp_remain + hp_remain_diff * fade_progress_), bar_height);
    painter.drawRect(hp_bar_diff);
    
    if (hp_remain <= 0.3f) {
        painter.setBrush(solid_red_);
        auto palette = hp_value_label_->palette();
        palette.setColor(QPalette::WindowText, solid_red_);
        hp_value_label_->setPalette(palette);
    } else {
        painter.setBrush(solid_);
        auto palette = hp_value_label_->palette();
        palette.setColor(QPalette::WindowText, solid_);
        hp_value_label_->setPalette(palette);
    }
    QRect  hp_bar_front(px, py, bar_width * hp_remain, bar_height);
    painter.drawRect(hp_bar_front);
}

void HUDHPPanel::setHP(int v) {
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << v;

    if (v == current_hp_) {
        return;
    }
    diff_hp_ = v - current_hp_;
    if (diff_hp_ < 0) {
        hp_diff_label_->setText(QString::fromStdString(std::to_string(diff_hp_)));
    } else {
        hp_diff_label_->setText("");
    }
    hp_value_label_->setText(QString::fromStdString(ss.str()));
    current_hp_ = v;
    start_point_ = std::chrono:: system_clock::now();
}

void HUDHPPanel::setMaxHP(int v) {
    max_hp_ = v;
}