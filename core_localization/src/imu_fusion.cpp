// 2 つの IMU の測定の融合。考え方はヘッダを参照。

#include "core_localization/imu_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>

namespace core_localization::imu_fusion
{
namespace
{
// 時刻がこれより大きく戻ったら、記録の再生のやり直しなどとみなして状態を捨てる。
// 小さな逆転は通信の順序の入れ替わりなので、その測定だけを捨てる
constexpr double kClockJumpS = 1.0;
// 相関を取るのに最低限必要な主の測定の数
constexpr std::size_t kMinCorrelationSamples = 20U;

double square(const double value) {return value * value;}

// 値が壊れているか飽和していて、使えない
bool isUnusable(const Eigen::Vector3d & value, const double saturation)
{
  return !value.allFinite() || isSaturated(value, saturation);
}

Contribution contributionOf(const bool primary, const bool secondary)
{
  if (primary && secondary) {
    return Contribution::kBoth;
  }
  return primary ? Contribution::kPrimary : Contribution::kSecondary;
}

double mean(const std::vector<double> & values)
{
  double sum = 0.0;
  for (const double value : values) {
    sum += value;
  }
  return sum / static_cast<double>(values.size());
}

double standardDeviation(const std::vector<double> & values)
{
  const double average = mean(values);
  double sum = 0.0;
  for (const double value : values) {
    sum += square(value - average);
  }
  return std::sqrt(sum / static_cast<double>(values.size()));
}

// ピアソンの相関係数。どちらかが一定なら 0
double correlation(const std::vector<double> & x, const std::vector<double> & y)
{
  const double mean_x = mean(x);
  const double mean_y = mean(y);
  double sxx = 0.0;
  double syy = 0.0;
  double sxy = 0.0;
  for (std::size_t i = 0U; i < x.size(); ++i) {
    const double dx = x[i] - mean_x;
    const double dy = y[i] - mean_y;
    sxx += dx * dx;
    syy += dy * dy;
    sxy += dx * dy;
  }
  if (sxx <= 0.0 || syy <= 0.0) {
    return 0.0;
  }
  return sxy / std::sqrt(sxx * syy);
}

// 時刻順の samples を、各 times - offset_s の時刻で線形補間する。samples は 2 つ以上
std::vector<double> resampleAt(
  const std::vector<RateSample> & samples, const std::vector<double> & times,
  const double offset_s)
{
  std::vector<double> values;
  values.reserve(times.size());
  std::size_t index = 0U;
  for (const double time : times) {
    const double target = time - offset_s;
    while (index + 2U < samples.size() && samples[index + 1U].stamp_s <= target) {
      ++index;
    }
    const auto & before = samples[index];
    const auto & after = samples[index + 1U];
    const double span = after.stamp_s - before.stamp_s;
    const double ratio = span > 0.0 ? std::clamp((target - before.stamp_s) / span, 0.0, 1.0) : 0.0;
    values.push_back(
      before.magnitude_radps + ratio * (after.magnitude_radps - before.magnitude_radps));
  }
  return values;
}

std::string positiveError(const std::string & name, const double value)
{
  std::ostringstream stream;
  stream << name << " は正の値にしてください（" << value << "）";
  return stream.str();
}
}  // namespace

bool isSaturated(const Eigen::Vector3d & value, const double threshold)
{
  return value.cwiseAbs().maxCoeff() >= threshold;
}

Eigen::Vector3d transferAcceleration(
  const Eigen::Vector3d & accel_at_secondary, const Eigen::Vector3d & gyro,
  const Eigen::Vector3d & angular_accel, const Eigen::Vector3d & lever_arm)
{
  return accel_at_secondary + angular_accel.cross(lever_arm) + gyro.cross(gyro.cross(lever_arm));
}

ImuSample interpolate(const ImuSample & before, const ImuSample & after, const double stamp_s)
{
  const double span = after.stamp_s - before.stamp_s;
  ImuSample sample = before;
  sample.stamp_s = stamp_s;
  if (span <= 0.0) {
    return sample;
  }
  const double ratio = (stamp_s - before.stamp_s) / span;
  sample.gyro = before.gyro + ratio * (after.gyro - before.gyro);
  sample.accel = before.accel + ratio * (after.accel - before.accel);
  return sample;
}

std::optional<Measurement> inverseVarianceMean(const std::vector<Measurement> & measurements)
{
  if (measurements.empty()) {
    return std::nullopt;
  }
  double weight_sum = 0.0;
  Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
  for (const auto & measurement : measurements) {
    const double weight = 1.0 / measurement.variance;
    weight_sum += weight;
    weighted_sum += weight * measurement.value;
  }
  return Measurement{weighted_sum / weight_sum, 1.0 / weight_sum};
}

bool isPlausibleGravity(const double mean_accel_norm_mps2, const double tolerance)
{
  return std::abs(mean_accel_norm_mps2 - kStandardGravity) <= tolerance * kStandardGravity;
}

UnitCheckState accumulateUnitCheck(
  UnitCheckState state, const Eigen::Vector3d & accel, const std::size_t samples,
  const double tolerance)
{
  if (state.result.status != UnitCheckStatus::kPending || !accel.allFinite()) {
    return state;
  }
  state.sum += accel;
  ++state.count;
  if (state.count >= samples) {
    state.result.mean_accel_norm_mps2 = (state.sum / static_cast<double>(state.count)).norm();
    state.result.status = isPlausibleGravity(state.result.mean_accel_norm_mps2, tolerance) ?
      UnitCheckStatus::kPassed : UnitCheckStatus::kFailed;
  }
  return state;
}

std::optional<std::string> validateParams(const FusionParams & params)
{
  const std::vector<std::pair<std::string, double>> positives{
    {"primary.gyro_noise_std_radps", params.primary.gyro_noise_std_radps},
    {"primary.accel_noise_std_mps2", params.primary.accel_noise_std_mps2},
    {"primary.gyro_saturation_radps", params.primary.gyro_saturation_radps},
    {"primary.accel_saturation_mps2", params.primary.accel_saturation_mps2},
    {"secondary.gyro_noise_std_radps", params.secondary.gyro_noise_std_radps},
    {"secondary.accel_noise_std_mps2", params.secondary.accel_noise_std_mps2},
    {"secondary.gyro_saturation_radps", params.secondary.gyro_saturation_radps},
    {"secondary.accel_saturation_mps2", params.secondary.accel_saturation_mps2},
    {"timeout_s", params.timeout_s},
    {"angular_accel_time_constant_s", params.angular_accel_time_constant_s},
    {"unit_check.tolerance", params.unit_check_tolerance},
    {"time_offset_estimation.window_s", params.offset_estimation.window_s},
    {"time_offset_estimation.search_range_s", params.offset_estimation.search_range_s},
    {"time_offset_estimation.step_s", params.offset_estimation.step_s},
    {"time_offset_estimation.interval_s", params.offset_estimation.interval_s},
  };
  for (const auto & [name, value] : positives) {
    if (!(value > 0.0)) {
      return positiveError(name, value);
    }
  }
  if (!(params.max_wait_s >= 0.0)) {
    return "max_wait_s は 0 以上にしてください";
  }
  const auto & estimation = params.offset_estimation;
  if (estimation.step_s > estimation.search_range_s) {
    std::ostringstream stream;
    stream << "time_offset_estimation.step_s（" << estimation.step_s << "）が search_range_s（"
           << estimation.search_range_s << "）より大きく、探す候補がありません";
    return stream.str();
  }
  if (estimation.window_s <= 2.0 * estimation.search_range_s) {
    std::ostringstream stream;
    stream << "time_offset_estimation.window_s（" << estimation.window_s
           << "）は search_range_s の 2 倍（" << 2.0 * estimation.search_range_s
           << "）より長くしてください。短いと相関を取る区間が残りません";
    return stream.str();
  }
  return std::nullopt;
}

OffsetEstimate estimateTimeOffset(
  const std::vector<RateSample> & primary, const std::vector<RateSample> & secondary,
  const double center_offset_s, const OffsetEstimationParams & params)
{
  OffsetEstimate result;
  result.offset_s = center_offset_s;
  if (primary.size() < kMinCorrelationSamples || secondary.size() < 2U) {
    return result;
  }
  const double low = center_offset_s - params.search_range_s;
  const double high = center_offset_s + params.search_range_s;
  // どの候補でも補助の測定の範囲に収まる主の測定だけを使う
  const double first =
    std::max(secondary.front().stamp_s + high, primary.back().stamp_s - params.window_s);
  const double last = secondary.back().stamp_s + low;
  std::vector<double> times;
  std::vector<double> values;
  for (const auto & sample : primary) {
    if (sample.stamp_s >= first && sample.stamp_s <= last) {
      times.push_back(sample.stamp_s);
      values.push_back(sample.magnitude_radps);
    }
  }
  result.samples = times.size();
  if (times.size() < kMinCorrelationSamples || times.back() - times.front() < 0.5 * params.window_s) {
    return result;
  }
  if (standardDeviation(values) < params.min_excitation_radps) {
    result.status = OffsetStatus::kLowExcitation;
    return result;
  }

  const auto steps = static_cast<int>(std::floor(params.search_range_s / params.step_s + 1e-9));
  std::vector<double> scores;
  scores.reserve(static_cast<std::size_t>(2 * steps + 1));
  for (int k = -steps; k <= steps; ++k) {
    scores.push_back(correlation(
        values, resampleAt(secondary, times, center_offset_s + k * params.step_s)));
  }
  const auto best = static_cast<std::size_t>(
    std::distance(scores.begin(), std::max_element(scores.begin(), scores.end())));
  result.correlation = scores[best];
  result.offset_s =
    center_offset_s + (static_cast<double>(best) - static_cast<double>(steps)) * params.step_s;
  // 端が最大なら、本当のずれは探した範囲の外にあるかもしれない
  if (best == 0U || best + 1U == scores.size()) {
    result.status = OffsetStatus::kAtSearchEdge;
    return result;
  }
  // 最大の点と前後の点に放物線を当て、頂点を刻みより細かく求める
  const double before = scores[best - 1U];
  const double after = scores[best + 1U];
  const double curvature = before - 2.0 * result.correlation + after;
  if (curvature < 0.0) {
    result.offset_s += 0.5 * (before - after) / curvature * params.step_s;
  }
  result.status = result.correlation >= params.min_correlation ?
    OffsetStatus::kAccepted : OffsetStatus::kLowCorrelation;
  return result;
}

ImuFusion::ImuFusion(const FusionParams & params, const Extrinsic & extrinsic)
: params_(params), extrinsic_(extrinsic), offset_s_(params.secondary_time_offset_s)
{
  if (const auto error = validateParams(params)) {
    throw std::invalid_argument(*error);
  }
  // 確認しない設定なら、最初から使える
  if (params_.unit_check_samples == 0U) {
    primary_check_.result.status = UnitCheckStatus::kPassed;
    secondary_check_.result.status = UnitCheckStatus::kPassed;
  }
}

void ImuFusion::setParams(const FusionParams & params)
{
  if (const auto error = validateParams(params)) {
    throw std::invalid_argument(*error);
  }
  // 設定したずれが変わったか、推定をやめたら、設定の値に戻す
  if (params.secondary_time_offset_s != params_.secondary_time_offset_s ||
    !params.offset_estimation.enabled)
  {
    offset_s_ = params.secondary_time_offset_s;
    last_estimate_.reset();
  }
  params_ = params;
}

std::optional<double> ImuFusion::clockGap() const
{
  if (secondary_.empty() || !newest_primary_stamp_s_.has_value()) {
    return std::nullopt;
  }
  return secondary_.back().stamp_s + offset_s_ - *newest_primary_stamp_s_;
}

std::vector<FusedSample> ImuFusion::addPrimary(const ImuSample & sample, const double now_s)
{
  std::vector<FusedSample> outputs;
  if (newest_primary_stamp_s_.has_value() && sample.stamp_s <= *newest_primary_stamp_s_) {
    if (*newest_primary_stamp_s_ - sample.stamp_s <= kClockJumpS) {
      return outputs;
    }
    reset();
  }
  primary_check_ = accumulateUnitCheck(
    primary_check_, sample.accel, params_.unit_check_samples, params_.unit_check_tolerance);
  newest_primary_stamp_s_ = sample.stamp_s;
  last_primary_arrival_s_ = now_s;
  pending_.push_back(sample);
  primary_rates_.push_back(RateSample{sample.stamp_s, sample.gyro.norm()});
  maybeEstimateOffset();
  resolvePending(outputs);
  trimHistory();
  return outputs;
}

std::vector<FusedSample> ImuFusion::addSecondary(const ImuSample & sample, const double now_s)
{
  std::vector<FusedSample> outputs;
  if (!secondary_.empty() && sample.stamp_s <= secondary_.back().stamp_s) {
    if (secondary_.back().stamp_s - sample.stamp_s <= kClockJumpS) {
      return outputs;
    }
    secondary_.clear();
  }
  secondary_check_ = accumulateUnitCheck(
    secondary_check_, sample.accel, params_.unit_check_samples, params_.unit_check_tolerance);
  secondary_.push_back(sample);
  resolvePending(outputs);
  // 主が途絶している間は、補助の時刻（ずれを補正した値）と周期で出す
  if (pending_.empty() && primaryTimedOut(now_s)) {
    const double stamp_s = sample.stamp_s + offset_s_;
    if (!last_output_stamp_s_.has_value() || stamp_s > *last_output_stamp_s_) {
      const auto value = secondaryValue(sample);
      emit(fuse(stamp_s, nullptr, &value), outputs);
    }
  }
  trimHistory();
  return outputs;
}

void ImuFusion::reset()
{
  pending_.clear();
  secondary_.clear();
  primary_rates_.clear();
  newest_primary_stamp_s_.reset();
  last_primary_arrival_s_.reset();
  last_output_stamp_s_.reset();
  next_estimation_stamp_s_.reset();
  last_gyro_.reset();
  angular_accel_.setZero();
}

ImuFusion::SecondaryValue ImuFusion::secondaryValue(const ImuSample & sample) const
{
  return SecondaryValue{
    sample,
    isUnusable(sample.gyro, params_.secondary.gyro_saturation_radps),
    isUnusable(sample.accel, params_.secondary.accel_saturation_mps2)};
}

ImuFusion::SecondaryLookup ImuFusion::lookupSecondary(const double stamp_s) const
{
  SecondaryLookup lookup;
  if (secondary_.empty()) {
    return lookup;
  }
  const double target = stamp_s - offset_s_;
  const double newest = secondary_.back().stamp_s;
  if (newest < target) {
    // 補助の測定がまだ主の時刻に届いていない。途絶していなければ、上限まで待つ
    const bool dropped = target - newest > params_.timeout_s;
    const bool waited_too_long = newest_primary_stamp_s_.has_value() &&
      *newest_primary_stamp_s_ - stamp_s >= params_.max_wait_s;
    if (!dropped && !waited_too_long) {
      lookup.status = LookupStatus::kWait;
    }
    return lookup;
  }
  const auto after = std::lower_bound(
    secondary_.begin(), secondary_.end(), target,
    [](const ImuSample & sample, const double time) {return sample.stamp_s < time;});
  if (after->stamp_s == target) {
    lookup.status = LookupStatus::kAvailable;
    lookup.value = secondaryValue(*after);
    lookup.value.sample.stamp_s = stamp_s;
    return lookup;
  }
  // 主の時刻より前の補助の測定がないか、間が欠けていれば補間しない
  if (after == secondary_.begin()) {
    return lookup;
  }
  const auto before = std::prev(after);
  if (after->stamp_s - before->stamp_s > params_.timeout_s) {
    return lookup;
  }
  const auto before_value = secondaryValue(*before);
  const auto after_value = secondaryValue(*after);
  lookup.status = LookupStatus::kAvailable;
  lookup.value.sample = interpolate(*before, *after, target);
  lookup.value.sample.stamp_s = stamp_s;
  lookup.value.gyro_saturated = before_value.gyro_saturated || after_value.gyro_saturated;
  lookup.value.accel_saturated = before_value.accel_saturated || after_value.accel_saturated;
  return lookup;
}

void ImuFusion::resolvePending(std::vector<FusedSample> & outputs)
{
  while (!pending_.empty()) {
    const ImuSample primary = pending_.front();
    // 主が途絶していた間に補助だけで先の時刻まで出していたら、戻らない
    if (last_output_stamp_s_.has_value() && primary.stamp_s <= *last_output_stamp_s_) {
      pending_.pop_front();
      continue;
    }
    const auto lookup = lookupSecondary(primary.stamp_s);
    if (lookup.status == LookupStatus::kWait) {
      return;
    }
    pending_.pop_front();
    emit(
      fuse(
        primary.stamp_s, &primary,
        lookup.status == LookupStatus::kAvailable ? &lookup.value : nullptr),
      outputs);
  }
}

std::optional<FusedSample> ImuFusion::fuse(
  const double stamp_s, const ImuSample * primary, const SecondaryValue * secondary)
{
  // 届いている側の単位の確認が済むまでは出さない。片側の確認が先に済んだ間だけ片側で出すと、
  // FAST-LIO の初期化（起動時の平均）に性質の違う値が混ざる。届かない側は待たない
  const bool primary_pending =
    primary != nullptr && primary_check_.result.status == UnitCheckStatus::kPending;
  const bool secondary_pending =
    secondary != nullptr && secondary_check_.result.status == UnitCheckStatus::kPending;
  if (primary_pending || secondary_pending) {
    return std::nullopt;
  }
  const bool use_primary =
    primary != nullptr && primary_check_.result.status == UnitCheckStatus::kPassed;
  const bool use_secondary =
    secondary != nullptr && secondary_check_.result.status == UnitCheckStatus::kPassed;

  const bool primary_gyro =
    use_primary && !isUnusable(primary->gyro, params_.primary.gyro_saturation_radps);
  const bool secondary_gyro = use_secondary && !secondary->gyro_saturated;
  std::vector<Measurement> gyros;
  if (primary_gyro) {
    gyros.push_back(Measurement{primary->gyro, square(params_.primary.gyro_noise_std_radps)});
  }
  if (secondary_gyro) {
    gyros.push_back(Measurement{
        extrinsic_.rotation * secondary->sample.gyro,
        square(params_.secondary.gyro_noise_std_radps)});
  }
  const auto gyro = inverseVarianceMean(gyros);
  if (!gyro.has_value()) {
    return std::nullopt;
  }
  const Eigen::Vector3d angular_accel = updateAngularAccel(stamp_s, gyro->value);

  const bool primary_accel =
    use_primary && !isUnusable(primary->accel, params_.primary.accel_saturation_mps2);
  const bool secondary_accel = use_secondary && !secondary->accel_saturated;
  std::vector<Measurement> accels;
  if (primary_accel) {
    accels.push_back(Measurement{primary->accel, square(params_.primary.accel_noise_std_mps2)});
  }
  if (secondary_accel) {
    accels.push_back(Measurement{
        transferAcceleration(
          extrinsic_.rotation * secondary->sample.accel, gyro->value, angular_accel,
          extrinsic_.lever_arm),
        square(params_.secondary.accel_noise_std_mps2)});
  }
  const auto accel = inverseVarianceMean(accels);
  if (!accel.has_value()) {
    return std::nullopt;
  }

  FusedSample fused;
  fused.stamp_s = stamp_s;
  fused.gyro = gyro->value;
  fused.accel = accel->value;
  fused.gyro_variance = gyro->variance;
  fused.accel_variance = accel->variance;
  fused.gyro_from = contributionOf(primary_gyro, secondary_gyro);
  fused.accel_from = contributionOf(primary_accel, secondary_accel);
  return fused;
}

Eigen::Vector3d ImuFusion::updateAngularAccel(const double stamp_s, const Eigen::Vector3d & gyro)
{
  if (last_gyro_.has_value()) {
    const double dt = stamp_s - last_gyro_->stamp_s;
    if (dt <= 0.0) {
      return angular_accel_;
    }
    if (dt > params_.timeout_s) {
      // 間が空いた差分は角加速度を表さないので、0 からやり直す
      angular_accel_.setZero();
    } else {
      const Eigen::Vector3d raw = (gyro - last_gyro_->gyro) / dt;
      const double gain = 1.0 - std::exp(-dt / params_.angular_accel_time_constant_s);
      angular_accel_ += gain * (raw - angular_accel_);
    }
  }
  last_gyro_ = GyroHistory{stamp_s, gyro};
  return angular_accel_;
}

void ImuFusion::emit(const std::optional<FusedSample> & sample, std::vector<FusedSample> & outputs)
{
  if (!sample.has_value()) {
    return;
  }
  last_output_stamp_s_ = sample->stamp_s;
  outputs.push_back(*sample);
}

bool ImuFusion::primaryTimedOut(const double now_s) const
{
  return !last_primary_arrival_s_.has_value() ||
         now_s - *last_primary_arrival_s_ > params_.timeout_s;
}

void ImuFusion::maybeEstimateOffset()
{
  const auto & estimation = params_.offset_estimation;
  if (!estimation.enabled || !newest_primary_stamp_s_.has_value()) {
    return;
  }
  if (next_estimation_stamp_s_.has_value() && *newest_primary_stamp_s_ < *next_estimation_stamp_s_) {
    return;
  }
  next_estimation_stamp_s_ = *newest_primary_stamp_s_ + estimation.interval_s;
  std::vector<RateSample> secondary_rates;
  secondary_rates.reserve(secondary_.size());
  std::transform(
    secondary_.begin(), secondary_.end(), std::back_inserter(secondary_rates),
    [](const ImuSample & sample) {return RateSample{sample.stamp_s, sample.gyro.norm()};});
  const auto estimate = estimateTimeOffset(
    std::vector<RateSample>(primary_rates_.begin(), primary_rates_.end()), secondary_rates,
    params_.secondary_time_offset_s, estimation);
  last_estimate_ = estimate;
  if (estimate.status == OffsetStatus::kAccepted) {
    offset_s_ = estimate.offset_s;
  }
}

double ImuFusion::historyLength() const
{
  const auto & estimation = params_.offset_estimation;
  // 補間と待ちに要る長さと、推定に要る長さの長い方。余裕を 1 秒足す
  const double interpolation = params_.timeout_s + params_.max_wait_s;
  const double for_estimation =
    estimation.enabled ? estimation.window_s + 2.0 * estimation.search_range_s : 0.0;
  return std::max(interpolation, for_estimation) + 1.0;
}

void ImuFusion::trimHistory()
{
  const double length = historyLength();
  if (!secondary_.empty()) {
    const double oldest = secondary_.back().stamp_s - length;
    while (secondary_.size() > 2U && secondary_.front().stamp_s < oldest) {
      secondary_.pop_front();
    }
  }
  if (newest_primary_stamp_s_.has_value()) {
    const double oldest = *newest_primary_stamp_s_ - length;
    while (!primary_rates_.empty() && primary_rates_.front().stamp_s < oldest) {
      primary_rates_.pop_front();
    }
  }
}
}  // namespace core_localization::imu_fusion
