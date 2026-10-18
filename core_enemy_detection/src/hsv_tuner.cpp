// 起動例: ros2 run core_enemy_detection hsv_tuner --ros-args
//   -p image_topic:=/raw_image -p detector:=/target_detector
// Qtは画面、rclcppは通信、OpenCVは色抽出を担当する。
#include <QApplication>
#include <QCheckBox>
#include <QFileDialog>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QPixmap>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>
#include <array>
#include <chrono>
#include <fstream>
#include <map>
#include <stdexcept>

using Values = std::map<std::string, std::vector<int64_t>>;
struct Range {
    const char *title, *lower, *upper;
    std::array<int, 3> low, high;
};
// 赤は色相の端をまたぐので、二つの範囲を最後にORで合成する。
const std::array<Range, 4> ranges{{
    {"Panel", "panel_range_lower", "panel_range_upper", {0,0,0}, {179,255,80}},
    {"Blue LED", "blue_range_lower", "blue_range_upper", {105,64,125}, {135,255,255}},
    {"Red LED 1", "red_range_lower1", "red_range_upper1", {0,100,125}, {10,255,255}},
    {"Red LED 2", "red_range_lower2", "red_range_upper2", {175,100,125}, {179,255,255}}
}};

class HsvTuner : public QWidget {
    using Get = rcl_interfaces::srv::GetParameters;
    using Set = rcl_interfaces::srv::SetParametersAtomically;
    using Clock = std::chrono::steady_clock;
public:
    explicit HsvTuner(const rclcpp::Node::SharedPtr &node) : node_(node) {
        const auto topic = node_->declare_parameter<std::string>("image_topic", "/raw_image");
        detector_ = node_->declare_parameter<std::string>("detector", "/target_detector");
        if (detector_.empty() || detector_[0] != '/') detector_ = "/" + detector_;
        while (detector_.size() > 1 && detector_.back() == '/') detector_.pop_back();
        setWindowTitle(QString::fromStdString("HSV Tuner — " + topic));
        auto *layout = new QVBoxLayout(this);
        info_ = new QLabel(QString::fromStdString("画像待ち: " + topic));
        layout->addWidget(info_);
        auto *images = new QHBoxLayout;
        const char *titles[] = {"Original", "Panel mask", "Blue mask", "Red mask (1 OR 2)"};
        for (size_t i = 0; i < views_.size(); ++i) {
            auto *box = new QVBoxLayout;
            box->addWidget(new QLabel(titles[i]));
            views_[i] = new QLabel;
            views_[i]->setMinimumSize(220, 180);
            views_[i]->setAlignment(Qt::AlignCenter);
            box->addWidget(views_[i]); images->addLayout(box);
        }
        layout->addLayout(images);
        auto *tabs = new QTabWidget;
        for (const auto &range : ranges) {
            auto *page = new QWidget;
            auto *grid = new QGridLayout(page);
            for (int side = 0; side < 2; ++side) {
                const std::string name = side == 0 ? range.lower : range.upper;
                const auto defaults = side == 0 ? range.low : range.high;
                for (int channel = 0; channel < 3; ++channel) {
                    const int row = side * 3 + channel;
                    grid->addWidget(new QLabel(QString(side == 0 ? "下限 " : "上限 ") + "HSV"[channel]), row, 0);
                    auto *slider = new QSlider(Qt::Horizontal);
                    auto *spin = new QSpinBox;
                    // OpenCVの8bit HSV: H=0..179、S/V=0..255。
                    slider->setRange(0, channel == 0 ? 179 : 255);
                    spin->setRange(0, channel == 0 ? 179 : 255);
                    connect(slider, &QSlider::valueChanged, spin, &QSpinBox::setValue);
                    connect(spin, qOverload<int>(&QSpinBox::valueChanged), slider, &QSlider::setValue);
                    spin->setValue(defaults[channel]);
                    controls_[name][channel] = spin;
                    grid->addWidget(slider, row, 1); grid->addWidget(spin, row, 2);
                }
            }
            tabs->addTab(page, range.title);
        }
        layout->addWidget(tabs);
        auto *buttons = new QHBoxLayout;
        freeze_ = new QCheckBox("プレビューを固定"); buttons->addWidget(freeze_);
        const auto button = [this, buttons](const char *text, auto callback) {
            auto *b = new QPushButton(text); buttons->addWidget(b);
            connect(b, &QPushButton::clicked, this, callback);
        };
        button("検出器から読込", [this] { fetch(); });
        button("検出器へ反映", [this] { apply(); });
        button("YAML読込", [this] { load(); });
        button("YAML保存", [this] { save(); });
        layout->addLayout(buttons);
        status_ = new QLabel("初期値は調整用。実行中の値は「検出器から読込」で取得します。");
        status_->setWordWrap(true); layout->addWidget(status_);
        get_ = node_->create_client<Get>(detector_ + "/get_parameters");
        set_ = node_->create_client<Set>(detector_ + "/set_parameters_atomically");
        // Best Effort購読は、bag中継のReliable配信も受信できる。
        sub_ = node_->create_subscription<sensor_msgs::msg::Image>(topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                ++received_; last_frame_ = Clock::now();
                if (freeze_->isChecked()) return;
                try {
                    // メッセージ破棄後も保持するためclone。最新の1枚だけ保存する。
                    frame_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
                } catch (const std::exception &e) { status_->setText(e.what()); }
            });
        // ROSとQtを同じスレッドで進める。別スレッドからGUIを操作しない。
        timer_ = new QTimer(this);
        connect(timer_, &QTimer::timeout, this, [this] { tick(); });
        timer_->start(50); resize(1050, 650);
    }
private:
    static void validate(const Values &values) {
        for (const auto &range : ranges) {
            for (const auto name : {range.lower, range.upper}) {
                const auto &v = values.at(name);
                if (v.size() != 3) throw std::runtime_error(std::string(name) + ": 整数3個が必要です");
                for (int i = 0; i < 3; ++i)
                    if (v[i] < 0 || v[i] > (i == 0 ? 179 : 255))
                        throw std::runtime_error(std::string(name) + ": HSV範囲外です");
            }
            for (int i = 0; i < 3; ++i)
                if (values.at(range.lower)[i] > values.at(range.upper)[i])
                    throw std::runtime_error(std::string(range.lower) + ": 下限は上限以下にしてください");
        }
    }
    Values values() const {
        Values result;
        for (const auto &entry : controls_)
            for (auto *spin : entry.second) result[entry.first].push_back(spin->value());
        validate(result); return result;
    }
    void setValues(const Values &v) {
        validate(v); // 全項目の検証後に反映し、部分的な変更を防ぐ。
        for (const auto &entry : controls_)
            for (int i = 0; i < 3; ++i) entry.second[i]->setValue(v.at(entry.first)[i]);
    }
    bool ready(bool available) {
        if (pending_) { status_->setText("通信中です。"); return false; }
        if (!available) { status_->setText(QString::fromStdString("検出器に接続できません: " + detector_)); return false; }
        pending_ = true; deadline_ = Clock::now() + std::chrono::seconds(5);
        status_->setText("通信中…"); return true;
    }
    void fetch() {
        if (!ready(get_->service_is_ready())) return;
        auto request = std::make_shared<Get::Request>();
        for (const auto &entry : controls_) request->names.push_back(entry.first);
        const auto generation = ++generation_;
        auto future = get_->async_send_request(request, [this, request, generation](rclcpp::Client<Get>::SharedFuture f) {
            if (generation != generation_) return;
            pending_ = false;
            try {
                Values v; const auto response = f.get();
                if (response->values.size() != request->names.size()) throw std::runtime_error("応答の項目数が不正です");
                for (size_t i = 0; i < request->names.size(); ++i) {
                    if (response->values[i].type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY)
                        throw std::runtime_error("HSVパラメータが存在しないか、型が違います");
                    v[request->names[i]] = response->values[i].integer_array_value;
                }
                setValues(v); status_->setText("検出器の値を読み込みました。");
            } catch (const std::exception &e) { status_->setText(e.what()); }
        });
        request_id_ = future.request_id; getting_ = true;
    }
    void apply() {
        try {
            auto request = std::make_shared<Set::Request>();
            for (const auto &entry : values())
                request->parameters.push_back(rclcpp::Parameter(entry.first, entry.second).to_parameter_msg());
            if (!ready(set_->service_is_ready())) return;
            const auto generation = ++generation_;
            // 上下限8配列を一括送信。検出器側が全て検証してから反映する。
            auto future = set_->async_send_request(request, [this, generation](rclcpp::Client<Set>::SharedFuture f) {
                if (generation != generation_) return;
                pending_ = false;
                try {
                    const auto result = f.get()->result;
                    status_->setText(result.successful ? "反映しました。" : QString::fromStdString("反映失敗: " + result.reason));
                } catch (const std::exception &e) { status_->setText(e.what()); }
            });
            request_id_ = future.request_id; getting_ = false;
        } catch (const std::exception &e) { status_->setText(e.what()); }
    }
    void load() {
        const auto path = QFileDialog::getOpenFileName(this, "HSV設定を読込", "", "YAML (*.yaml *.yml)");
        if (path.isEmpty()) return;
        try {
            const auto doc = YAML::LoadFile(path.toStdString());
            const auto block = doc[detector_] ? doc[detector_] : doc["/**"];
            Values v;
            for (const auto &entry : controls_)
                v[entry.first] = block["ros__parameters"][entry.first].as<std::vector<int64_t>>();
            setValues(v); status_->setText("読込済み。検出器への反映は別操作です。");
        } catch (const std::exception &e) { status_->setText(e.what()); }
    }
    void save() {
        try {
            const auto v = values();
            const auto path = QFileDialog::getSaveFileName(this, "HSV設定を保存", "hsv_params.yaml", "YAML (*.yaml)");
            if (path.isEmpty()) return;
            // ros2 --params-fileで読める形式。既存設定への追記ではなく書き出し。
            YAML::Node doc;
            for (const auto &entry : v) doc[detector_]["ros__parameters"][entry.first] = entry.second;
            std::ofstream stream(path.toStdString()); stream << doc; stream.close();
            if (!stream) throw std::runtime_error("保存に失敗しました");
            status_->setText("保存しました。");
        } catch (const std::exception &e) { status_->setText(e.what()); }
    }
    void tick() {
        // Ctrl+CでROSが終了したらQtのイベントループも閉じる。
        if (!rclcpp::ok()) { QApplication::quit(); return; }
        rclcpp::spin_some(node_);
        if (pending_ && Clock::now() > deadline_) {
            // タイムアウト時は要求を解放。遅れて届いた応答も世代番号で無視する。
            if (getting_) get_->remove_pending_request(request_id_);
            else set_->remove_pending_request(request_id_);
            ++generation_; pending_ = false;
            status_->setText("通信タイムアウト。反映状況は再読込して確認してください。");
        }
        if (frame_.empty()) return;
        info_->setText(QString("受信: %1枚 / %2×%3 / 最終受信 %4秒前（色抽出のみ）")
            .arg(received_).arg(frame_.cols).arg(frame_.rows)
            .arg(std::chrono::duration<double>(Clock::now()-last_frame_).count(), 0, 'f', 1));
        try {
            const auto v = values();
            cv::Mat hsv; cv::cvtColor(frame_, hsv, cv::COLOR_BGR2HSV);
            std::array<cv::Mat, 4> mask, pictures;
            for (size_t i = 0; i < ranges.size(); ++i) {
                const auto &lo = v.at(ranges[i].lower), &hi = v.at(ranges[i].upper);
                cv::inRange(hsv, cv::Scalar(lo[0],lo[1],lo[2]), cv::Scalar(hi[0],hi[1],hi[2]), mask[i]);
            }
            // ノイズ除去と上下判定は検出器側の役割。この画面は色マスクを比較する。
            cv::cvtColor(frame_, pictures[0], cv::COLOR_BGR2RGB);
            cv::cvtColor(mask[0], pictures[1], cv::COLOR_GRAY2RGB);
            cv::cvtColor(mask[1], pictures[2], cv::COLOR_GRAY2RGB);
            cv::Mat red; cv::bitwise_or(mask[2],mask[3],red);
            cv::cvtColor(red, pictures[3], cv::COLOR_GRAY2RGB);
            for (size_t i = 0; i < pictures.size(); ++i) {
                const auto &p = pictures[i];
                // QImageはMatのメモリを参照するのでcopyして寿命を独立させる。
                QImage image(p.data,p.cols,p.rows,static_cast<int>(p.step),QImage::Format_RGB888);
                views_[i]->setPixmap(QPixmap::fromImage(image.copy()).scaled(250,220,Qt::KeepAspectRatio));
            }
        } catch (const std::exception &e) { status_->setText(e.what()); }
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Client<Get>::SharedPtr get_;
    rclcpp::Client<Set>::SharedPtr set_;
    std::string detector_;
    std::map<std::string,std::array<QSpinBox*,3>> controls_;
    std::array<QLabel*,4> views_;
    QLabel *info_, *status_;
    QCheckBox *freeze_;
    QTimer *timer_;
    cv::Mat frame_;
    Clock::time_point last_frame_, deadline_;
    qulonglong received_ = 0;
    uint64_t generation_ = 0;
    int64_t request_id_ = 0;
    bool pending_ = false, getting_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // ROS引数をQtに渡さない。GUIはノードより先に破棄する。
    int qt_argc = 1; QApplication app(qt_argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hsv_tuner");
    int result;
    { HsvTuner window(node); window.show(); result = app.exec(); }
    node.reset(); rclcpp::shutdown(); return result;
}
