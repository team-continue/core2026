// 2 つの IMU の測定を、測定の段階で 1 つにまとめる処理。ROS に依存しない。
//
// 主の IMU の時刻・座標系・周期で出す。補助の IMU の測定は次の順に直して主と平均する。
// 1. 固定の外部パラメータで主の座標系へ回す
// 2. 時刻のずれを補正し、主の時刻に線形補間する
// 3. 加速度を剛体の式で主の位置へ移す
//      a_P = a_A + α × r + ω × (ω × r)
//    r は補助から主への位置、ω は融合した角速度、α はその差分を平滑化した角加速度
// 平均の重みは雑音の分散の逆数。飽和した量や途絶した側は使わず、残った側だけで出す。
//
// EKF などで平滑化した値は出さない。FAST-LIO は入力の IMU の雑音を、時間方向に相関のない白色雑音と
// 仮定して共分散を伝播する。平滑化した値は誤差が時間方向に相関するので、FAST-LIO が実際より小さい
// 不確かさを信じて自信過剰になり、LiDAR の観測で直すべきずれを直しにくくなる。平滑化は遅れも生む。
// そこで融合は測定ごとの重み付き平均にとどめ、バイアスの推定は FAST-LIO に任せる。
// 時刻を合わせる補間と、てこの腕の補正に使う α の平滑化は、この方針の例外として最小限にしている。

#ifndef CORE_LOCALIZATION__IMU_FUSION_HPP_
#define CORE_LOCALIZATION__IMU_FUSION_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace core_localization::imu_fusion
{
// 標準重力 [m/s2]。単位の確認と、g 単位の IMU の倍率に使う
constexpr double kStandardGravity = 9.80665;

// 1 つの IMU の測定。単位は rad/s と m/s2、座標系はその IMU のもの
struct ImuSample
{
  double stamp_s{0.0};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
};

// IMU ごとの性質
struct SensorParams
{
  // 1 サンプルの雑音の標準偏差。平均の重みは分散の逆数
  double gyro_noise_std_radps{0.002};
  double accel_noise_std_mps2{0.02};
  // どれかの軸の絶対値がこの値以上なら飽和とみなし、その量を使わない
  double gyro_saturation_radps{34.0};
  double accel_saturation_mps2{150.0};
};

// 角速度の大きさの相互相関で、補助の時刻のずれを推定する設定
struct OffsetEstimationParams
{
  bool enabled{false};
  // 相関を取る長さ
  double window_s{5.0};
  // 設定したずれ（FusionParams::secondary_time_offset_s）を中心に探す幅（片側）
  double search_range_s{0.05};
  // 探す刻み。最大の点の前後で放物線を当てて刻みより細かく求める
  double step_s{0.0005};
  // この相関係数より小さければ採用しない
  double min_correlation{0.9};
  // 主の角速度の大きさの標準偏差がこれより小さければ、動きが足りないとして推定しない
  double min_excitation_radps{0.2};
  // 推定する間隔（主の時刻）
  double interval_s{1.0};
};

struct FusionParams
{
  SensorParams primary;
  SensorParams secondary;
  // 測定の間隔がこれを超えたら途絶とみなす
  double timeout_s{0.05};
  // 補助の測定が主の時刻を超えるまで待つ上限（主の時刻で測る）。超えたら主だけで出す
  double max_wait_s{0.02};
  // 補助の時刻に足すと主の時刻になる値
  double secondary_time_offset_s{0.0};
  // 角加速度（角速度の差分）を平滑化する一次遅れの時定数
  double angular_accel_time_constant_s{0.02};
  OffsetEstimationParams offset_estimation;
  // 単位の確認に使う、起動直後の測定の数。0 なら確認しない
  std::size_t unit_check_samples{200};
  // 起動直後の加速度の大きさと重力の差の許容（重力に対する比）
  double unit_check_tolerance{0.3};
};

// 補助の IMU から主の IMU への固定の変換
struct Extrinsic
{
  // 補助の座標系のベクトルを主の座標系へ回す
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  // 補助の位置から主の位置へのベクトル（主の座標系）
  Eigen::Vector3d lever_arm{Eigen::Vector3d::Zero()};
};

// 出力の量ごとに、使った IMU
enum class Contribution : std::uint8_t {kPrimary, kSecondary, kBoth};

struct FusedSample
{
  double stamp_s{0.0};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
  // 各軸の分散（重み付き平均の分散）
  double gyro_variance{0.0};
  double accel_variance{0.0};
  Contribution gyro_from{Contribution::kBoth};
  Contribution accel_from{Contribution::kBoth};
};

// 分散付きの測定
struct Measurement
{
  Eigen::Vector3d value{Eigen::Vector3d::Zero()};
  double variance{1.0};
};

// 角速度の大きさの時系列
struct RateSample
{
  double stamp_s{0.0};
  double magnitude_radps{0.0};
};

enum class OffsetStatus : std::uint8_t
{
  kAccepted,
  kNotEnoughData,
  kLowExcitation,
  kLowCorrelation,
  kAtSearchEdge,
};

struct OffsetEstimate
{
  OffsetStatus status{OffsetStatus::kNotEnoughData};
  // 補助の時刻に足すと主の時刻になる値
  double offset_s{0.0};
  double correlation{0.0};
  std::size_t samples{0U};
};

enum class UnitCheckStatus : std::uint8_t {kPending, kPassed, kFailed};

struct UnitCheckResult
{
  UnitCheckStatus status{UnitCheckStatus::kPending};
  // 確認に使った測定の加速度の平均の大きさ [m/s2]
  double mean_accel_norm_mps2{0.0};
};

// 起動直後の加速度を溜めて、単位の倍率を確かめる途中の状態
struct UnitCheckState
{
  std::size_t count{0U};
  Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
  UnitCheckResult result;
};

// 加速度を 1 つ加えた状態を返す。samples 個そろったら、平均の大きさを重力と比べて結果を決める。
// 単位の倍率を誤ると（g 単位の IMU に倍率 1 など）、ここで重力から大きく外れる
UnitCheckState accumulateUnitCheck(
  UnitCheckState state, const Eigen::Vector3d & accel, std::size_t samples, double tolerance);

// どれかの軸の絶対値が閾値以上か
bool isSaturated(const Eigen::Vector3d & value, double threshold);

// 補助の位置の加速度を、剛体の式で主の位置の加速度に直す。入力はすべて主の座標系
Eigen::Vector3d transferAcceleration(
  const Eigen::Vector3d & accel_at_secondary, const Eigen::Vector3d & gyro,
  const Eigen::Vector3d & angular_accel, const Eigen::Vector3d & lever_arm);

// before と after の間の stamp_s の値を線形補間する。2 つの時刻が同じなら before の値を使う
ImuSample interpolate(const ImuSample & before, const ImuSample & after, double stamp_s);

// 分散の逆数を重みにした平均。空なら値なし
std::optional<Measurement> inverseVarianceMean(const std::vector<Measurement> & measurements);

// 起動直後の加速度の平均の大きさが、重力に近いか
bool isPlausibleGravity(double mean_accel_norm_mps2, double tolerance);

// 設定の組み合わせの誤りを、理由の文で返す。正しければ値なし
std::optional<std::string> validateParams(const FusionParams & params);

// 主の時刻 t の補助の値を s(t - offset) として、角速度の大きさの相関係数が最大になる offset を探す。
// 大きさを比べるので、取り付けの向き（外部パラメータの回転）に左右されない
OffsetEstimate estimateTimeOffset(
  const std::vector<RateSample> & primary, const std::vector<RateSample> & secondary,
  double center_offset_s, const OffsetEstimationParams & params);

// 主と補助の測定を受け取り、融合した測定を時刻順に出す
class ImuFusion
{
public:
  ImuFusion(const FusionParams & params, const Extrinsic & extrinsic);

  // 測定を加え、出せるようになった融合の結果を時刻順に返す。
  // now_s は受信した時刻（ノードの時計）で、主の途絶の判定に使う。
  // 測定の時刻だけで判定すると、補助の時刻が大きくずれているときに主が途絶したと誤る
  std::vector<FusedSample> addPrimary(const ImuSample & sample, double now_s);
  std::vector<FusedSample> addSecondary(const ImuSample & sample, double now_s);

  // 実行中に設定を変える。secondary_time_offset_s が変わったら推定の結果を捨てる
  void setParams(const FusionParams & params);

  // 今使っている補助の時刻のずれ
  double timeOffset() const {return offset_s_;}
  const std::optional<OffsetEstimate> & lastOffsetEstimate() const {return last_estimate_;}
  UnitCheckResult primaryUnitCheck() const {return primary_check_.result;}
  UnitCheckResult secondaryUnitCheck() const {return secondary_check_.result;}
  // 補助の最新の時刻（ずれを補正した値）から主の最新の時刻を引いた値
  std::optional<double> clockGap() const;

private:
  enum class LookupStatus : std::uint8_t {kAvailable, kWait, kUnavailable};

  struct SecondaryValue
  {
    ImuSample sample;
    bool gyro_saturated{false};
    bool accel_saturated{false};
  };

  struct SecondaryLookup
  {
    LookupStatus status{LookupStatus::kUnavailable};
    SecondaryValue value;
  };

  struct GyroHistory
  {
    double stamp_s{0.0};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  };

  void reset();
  SecondaryValue secondaryValue(const ImuSample & sample) const;
  SecondaryLookup lookupSecondary(double stamp_s) const;
  void resolvePending(std::vector<FusedSample> & outputs);
  std::optional<FusedSample> fuse(
    double stamp_s, const ImuSample * primary, const SecondaryValue * secondary);
  Eigen::Vector3d updateAngularAccel(double stamp_s, const Eigen::Vector3d & gyro);
  void emit(const std::optional<FusedSample> & sample, std::vector<FusedSample> & outputs);
  bool primaryTimedOut(double now_s) const;
  void maybeEstimateOffset();
  void trimHistory();
  double historyLength() const;

  FusionParams params_;
  Extrinsic extrinsic_;
  double offset_s_{0.0};
  // 補助の測定が届くのを待っている主の測定
  std::deque<ImuSample> pending_;
  // 補助の測定（時刻はずれを補正する前の値）
  std::deque<ImuSample> secondary_;
  // 時刻のずれの推定に使う主の角速度の大きさ
  std::deque<RateSample> primary_rates_;
  std::optional<double> newest_primary_stamp_s_;
  std::optional<double> last_primary_arrival_s_;
  std::optional<double> last_output_stamp_s_;
  std::optional<double> next_estimation_stamp_s_;
  std::optional<OffsetEstimate> last_estimate_;
  std::optional<GyroHistory> last_gyro_;
  Eigen::Vector3d angular_accel_{Eigen::Vector3d::Zero()};
  UnitCheckState primary_check_;
  UnitCheckState secondary_check_;
};
}  // namespace core_localization::imu_fusion

#endif  // CORE_LOCALIZATION__IMU_FUSION_HPP_
