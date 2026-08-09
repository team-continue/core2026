#include "SettingMenu.hpp"


SettingMenu::SettingMenu(std::shared_ptr<HUDManager> manager, QWidget *parent)
 :  QWidget(parent),
    manager_(manager)
{
    QPalette palette;

    layout_ = new QGridLayout(this);
    label_title_ = new QLabel("SETTING");
    label_aim_sensitivity_ = new QLabel("Aim Sensitivity");
    slider_aim_sensitivity_ = new QSlider(Qt::Horizontal);
    value_aim_sensitivity_ = new QLabel("0.00");
    label_max_velocity_ = new QLabel("Max Move Speed");
    slider_max_velocity_ = new QSlider(Qt::Horizontal);
    value_max_velocity_ = new QLabel("0.00");
    label_max_omega_ = new QLabel("Max Rotate Speed");
    slider_max_omega_ = new QSlider(Qt::Horizontal);
    value_max_omega_ = new QLabel("0.00");
    label_max_shooter_ = new QLabel("Max Shooter Speed");
    slider_max_shooter_ = new QSlider(Qt::Horizontal);
    value_max_shooter_ = new QLabel("0.00");
    label_team_ = new QLabel("Team");
    switch_team_ = new ToggleSwitch();
    value_team_ = new QLabel("");
    label_auto_aim_ = new QLabel("Auto Aim");
    switch_auto_aim_ = new ToggleSwitch();
    value_auto_aim_ = new QLabel("");

    // item設定
    items.emplace_back(std::make_tuple(label_aim_sensitivity_, slider_aim_sensitivity_, value_aim_sensitivity_));
    items.emplace_back(std::make_tuple(label_max_velocity_, slider_max_velocity_, value_max_velocity_));
    items.emplace_back(std::make_tuple(label_max_omega_, slider_max_omega_, value_max_omega_));
    items.emplace_back(std::make_tuple(label_max_shooter_, slider_max_shooter_, value_max_shooter_));
    items.emplace_back(std::make_tuple(label_team_, switch_team_, value_team_));
    items.emplace_back(std::make_tuple(label_auto_aim_, switch_auto_aim_, value_auto_aim_));

    // Qt Find用名前設定
    setObjectName("SettingMenu");

    // 各ラベルのフォント、サイズ、色を設定
    label_title_->setFont(QFont(USE_FONT, 20));
    palette = label_title_->palette(); palette.setColor(QPalette::WindowText, solid); label_title_->setPalette(palette);

    label_aim_sensitivity_->setFont(QFont(USE_FONT, 10));
    palette = label_aim_sensitivity_->palette(); palette.setColor(QPalette::WindowText, solid); label_aim_sensitivity_->setPalette(palette);
    value_aim_sensitivity_->setFont(QFont(USE_FONT, 10));
    palette = value_aim_sensitivity_->palette(); palette.setColor(QPalette::WindowText, solid); value_aim_sensitivity_->setPalette(palette);
    
    label_max_velocity_->setFont(QFont(USE_FONT, 10));
    palette = label_max_velocity_->palette(); palette.setColor(QPalette::WindowText, solid); label_max_velocity_->setPalette(palette);
    value_max_velocity_->setFont(QFont(USE_FONT, 10));
    palette = value_max_velocity_->palette(); palette.setColor(QPalette::WindowText, solid); value_max_velocity_->setPalette(palette);
    
    label_max_omega_->setFont(QFont(USE_FONT, 10));
    palette = label_max_omega_->palette(); palette.setColor(QPalette::WindowText, solid); label_max_omega_->setPalette(palette);
    value_max_omega_->setFont(QFont(USE_FONT, 10));
    palette = value_max_omega_->palette(); palette.setColor(QPalette::WindowText, solid); value_max_omega_->setPalette(palette);
    
    label_max_shooter_->setFont(QFont(USE_FONT, 10));
    palette = label_max_shooter_->palette(); palette.setColor(QPalette::WindowText, solid); label_max_shooter_->setPalette(palette);
    value_max_shooter_->setFont(QFont(USE_FONT, 10));
    palette = value_max_shooter_->palette(); palette.setColor(QPalette::WindowText, solid); value_max_shooter_->setPalette(palette);
    
    label_team_->setFont(QFont(USE_FONT, 10));
    palette = label_team_->palette(); palette.setColor(QPalette::WindowText, solid); label_team_->setPalette(palette);
    label_auto_aim_->setFont(QFont(USE_FONT, 10));
    palette = label_auto_aim_->palette(); palette.setColor(QPalette::WindowText, solid); label_auto_aim_->setPalette(palette);
    
    // レイアウト設定
    layout_->setColumnMinimumWidth(0, 400);
    layout_->setColumnMinimumWidth(1, 300);
    
    layout_->addWidget(label_title_,  0, 0, 1, 2);

    layout_->addWidget(label_aim_sensitivity_,  1, 0, 2, 1);
    layout_->addWidget(value_aim_sensitivity_,  1, 1, Qt::AlignCenter);
    layout_->addWidget(slider_aim_sensitivity_, 2, 1);
    
    layout_->addWidget(label_max_velocity_,     3, 0, 2, 1);
    layout_->addWidget(value_max_velocity_,     3, 1, Qt::AlignCenter);
    layout_->addWidget(slider_max_velocity_,    4, 1);
    
    layout_->addWidget(label_max_omega_,        5, 0, 2, 1);
    layout_->addWidget(value_max_omega_,        5, 1, Qt::AlignCenter);
    layout_->addWidget(slider_max_omega_,       6, 1);
    
    layout_->addWidget(label_max_shooter_,      7, 0, 2, 1);
    layout_->addWidget(value_max_shooter_,      7, 1, Qt::AlignCenter);
    layout_->addWidget(slider_max_shooter_,     8, 1);
    
    switch_team_->setOffColor(QColor(255, 50, 50));
    switch_team_->setOnColor(QColor(50, 50, 255));
    layout_->addWidget(label_team_,             9, 0, 2, 1);
    layout_->addWidget(switch_team_,            9, 1, 1, 1, Qt::AlignCenter);
    
    layout_->addWidget(label_auto_aim_,         11, 0, 2, 1);
    layout_->addWidget(switch_auto_aim_,        11, 1, 1, 1, Qt::AlignCenter);
    
    setLayout(layout_);
    
    // 背景設定
    palette = this->palette();
    palette.setColor(QPalette::Window, translucent_black);
    setPalette(palette);
    setAutoFillBackground(true);

    // 画面を中央に移動
    adjustSize();
    move(parentWidget()->width() / 2 - width() / 2, parentWidget()->height() / 2 - height() / 2);

    setRangeAimSensitivity(0, 100);
    setRangeMaxVelocity(0, 100);
    setRangeMaxOmega(0, 100);
    setRangeMaxShooter(0, 100);
}

void SettingMenu::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);

    // 選択されている項目をハイライトする
    QColor cursol_color = translucent;
    painter.fillRect(0, std::get<0>(items[cursor_pos])->y(), width(), std::get<0>(items[cursor_pos])->height(), cursol_color);
}

void SettingMenu::setParam(BattleParam &param) {
    slider_aim_sensitivity_->setValue(param.data.sensitivity * 1000);
    value_aim_sensitivity_->setText(QString::number(slider_aim_sensitivity_->value() * 0.001, 'f', 2));
    
    slider_max_velocity_->setValue(param.data.max_vel * 1000);
    value_max_velocity_->setText(QString::number(slider_max_velocity_->value() * 0.001, 'f', 2));
    
    slider_max_omega_->setValue(param.data.max_omega * 1000);
    value_max_omega_->setText(QString::number(slider_max_omega_->value() * 0.001, 'f', 2));

    slider_max_shooter_->setValue(param.data.max_roller_speed * 1000);
    value_max_shooter_->setText(QString::number(slider_max_shooter_->value() * 0.001, 'f', 2));
    
    switch_team_->setToggle(param.data.team);
    switch_auto_aim_->setToggle(param.data.enable_auto_aim);
}

void SettingMenu::open() {
    cursor_pos = 0;
    show();
}

void SettingMenu::inputCursorNext() {
    if (isHidden()) return;

    cursor_pos++;
    cursor_pos = cursor_pos % items.size();
}

void SettingMenu::inputCursorPrev() {
    if (isHidden()) return;

    cursor_pos--;
    cursor_pos = (cursor_pos + items.size()) % items.size();
}

void SettingMenu::inputCursorOK() {
    if (isHidden()) return;
    
    // 内部リクエスト通知
    BattleParamData param_data;
    param_data.sensitivity = slider_aim_sensitivity_->value() / 1000.0;
    param_data.max_vel = slider_max_velocity_->value() / 1000.0;
    param_data.max_omega = slider_max_omega_->value() / 1000.0;
    param_data.max_roller_speed = slider_max_shooter_->value() / 1000.0;
    param_data.team = switch_team_->isChecked() ? 1 : 0;
    param_data.enable_auto_aim = switch_auto_aim_->isChecked();
    
    // パラメータリクエスト、保存
    manager_->setBattleParameter(param_data);

    // 終了処理
    hide();
}

void SettingMenu::inputCursorBack() {
    if (isHidden()) return;

    // 変更値ロールバック処理
    // 本体のパラメータは変更しないので、ロールバックの処理とかいらないです。

    // 終了処理
    hide();
}

void SettingMenu::inputValueUp() {
    if (isHidden()) return;

    if (isSelectedSlider()) {
        auto slider = dynamic_cast<QSlider*>(std::get<1>(items[cursor_pos]));
        slider->setValue((double)slider->value() + slider->maximum() * 0.05);
        auto label = dynamic_cast<QLabel*>(std::get<2>(items[cursor_pos]));
        label->setText(QString::number(slider->value() * 0.001, 'f', 2));
    } else {
        switch (cursor_pos)
        {
        case SettingItem::Team:
        case SettingItem::AutoAim:
            dynamic_cast<ToggleSwitch*>(std::get<1>(items[cursor_pos]))->doToggle();
            break;
        default:
            break;
        }
    }
}

void SettingMenu::inputValueDown() {
    if (isHidden()) return;

    if (isSelectedSlider()) {
        auto slider = dynamic_cast<QSlider*>(std::get<1>(items[cursor_pos]));
        slider->setValue((double)slider->value() - slider->maximum() * 0.05);
        auto label = dynamic_cast<QLabel*>(std::get<2>(items[cursor_pos]));
        label->setText(QString::number(slider->value() * 0.001, 'f', 2));
    } else {
        switch (cursor_pos)
        {
        case SettingItem::Team:
        case SettingItem::AutoAim:
            dynamic_cast<ToggleSwitch*>(std::get<1>(items[cursor_pos]))->doToggle();
            break;
        default:
            break;
        }
    }
}

void SettingMenu::setRangeAimSensitivity(double min, double max) {
    auto slider = dynamic_cast<QSlider*>(std::get<1>(items[SettingItem::AimSensitivity]));
    slider->setMaximum(max * 1000);
    slider->setMinimum(min * 1000);
}

void SettingMenu::setRangeMaxVelocity(double min, double max) {
    auto slider = dynamic_cast<QSlider*>(std::get<1>(items[SettingItem::MaxVelocity]));
    slider->setMaximum(max * 1000);
    slider->setMinimum(min * 1000);
}

void SettingMenu::setRangeMaxOmega(double min, double max) {
    auto slider = dynamic_cast<QSlider*>(std::get<1>(items[SettingItem::MaxOmega]));
    slider->setMaximum(max * 1000);
    slider->setMinimum(min * 1000);
}

void SettingMenu::setRangeMaxShooter(double min, double max) {
    auto slider = dynamic_cast<QSlider*>(std::get<1>(items[SettingItem::MaxShooterSpeed]));
    slider->setMaximum(max * 1000);
    slider->setMinimum(min * 1000);
}