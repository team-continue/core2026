// core_shooter::TurretEnvelope の単体テスト。
//
// ヨー角ごとの許容ピッチ範囲（包絡線）の順引き・逆引きと、
// 掃過範囲を考慮した制限を ROS 起動なしで検証する。
//
// ここは機構干渉に直結するため、統合テストより細かく網羅する。

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "core_shooter/turret_envelope.hpp"

using core_shooter::EnvelopeResult;
using core_shooter::MotorLimits;
using core_shooter::PitchRange;
using core_shooter::TurretEnvelope;
using core_shooter::YawRange;

namespace
{

class TempYaml
{
public:
  explicit TempYaml(const std::string & body)
  {
    static int counter = 0;
    path_ = (std::filesystem::temp_directory_path() /
      ("core_test_envelope_" + std::to_string(++counter) + ".yaml")).string();
    std::ofstream out(path_);
    out << body;
  }
  ~TempYaml() {std::remove(path_.c_str());}
  TempYaml(const TempYaml &) = delete;
  TempYaml & operator=(const TempYaml &) = delete;
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

/// 右砲塔と同じ形。yaw 0.25〜0.30 で上限が 3.14 から 0.0 へ傾斜する。
constexpr const char * kRightLike =
  "envelope:\n"
  "  -0.48: {pitch_min: -1.8, pitch_max: 3.14}\n"
  "  0.25: {pitch_min: -1.8, pitch_max: 3.14}\n"
  "  0.30: {pitch_min: -1.8, pitch_max: 0.0}\n"
  "  2.20: {pitch_min: -1.8, pitch_max: 0.0}\n";

TurretEnvelope loadOrFail(const std::string & path)
{
  TurretEnvelope envelope;
  std::string reason;
  EXPECT_TRUE(TurretEnvelope::load(path, envelope, reason)) << reason;
  MotorLimits motor;
  motor.yaw_min = -3.14;
  motor.yaw_max = 3.14;
  motor.pitch_min = -3.14;
  motor.pitch_max = 3.14;
  envelope.setMotorLimits(motor);
  envelope.setPitchMargin(0.0);
  return envelope;
}

}  // namespace

// ---------------------------------------------------------------------------
// 読み込み
// ---------------------------------------------------------------------------

TEST(TurretEnvelope, LoadsAndSortsPoints)
{
  const TempYaml yaml(
    "envelope:\n"
    "  2.20: {pitch_min: -1.8, pitch_max: 0.0}\n"
    "  -0.48: {pitch_min: -1.8, pitch_max: 3.14}\n");
  const TurretEnvelope envelope = loadOrFail(yaml.path());
  EXPECT_EQ(envelope.size(), 2u);
  EXPECT_DOUBLE_EQ(envelope.yawMin(), -0.48);
  EXPECT_DOUBLE_EQ(envelope.yawMax(), 2.20);
}

TEST(TurretEnvelope, RejectsBrokenConfigurations)
{
  struct Case
  {
    const char * name;
    const char * body;
  };
  const Case cases[] = {
    {"envelope キーがない", "other:\n  0.0: {pitch_min: 0.0, pitch_max: 1.0}\n"},
    {"エントリがマップでない", "envelope:\n  0.0: 5\n"},
    {"pitch_max が欠けている", "envelope:\n  0.0: {pitch_min: 0.0}\n"},
    {"点が1つしかない", "envelope:\n  0.0: {pitch_min: 0.0, pitch_max: 1.0}\n"},
    {"min > max", "envelope:\n  0.0: {pitch_min: 1.0, pitch_max: 0.0}\n"
      "  1.0: {pitch_min: 0.0, pitch_max: 1.0}\n"},
    {"空", "envelope: {}\n"},
  };
  for (const Case & c : cases) {
    const TempYaml yaml(c.body);
    TurretEnvelope envelope;
    std::string reason;
    EXPECT_FALSE(TurretEnvelope::load(yaml.path(), envelope, reason)) << c.name;
    EXPECT_FALSE(reason.empty()) << c.name;
  }
}

TEST(TurretEnvelope, RejectsMissingFile)
{
  TurretEnvelope envelope;
  std::string reason;
  EXPECT_FALSE(TurretEnvelope::load("/nonexistent/envelope.yaml", envelope, reason));
}

// ---------------------------------------------------------------------------
// 順引き: ヨー -> ピッチ範囲
// ---------------------------------------------------------------------------

TEST(TurretEnvelope, PitchRangeAtGridPoints)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  EXPECT_DOUBLE_EQ(envelope.pitchRangeAt(0.25).max, 3.14);
  EXPECT_DOUBLE_EQ(envelope.pitchRangeAt(0.30).max, 0.0);
  EXPECT_DOUBLE_EQ(envelope.pitchRangeAt(2.20).max, 0.0);
  EXPECT_DOUBLE_EQ(envelope.pitchRangeAt(0.0).min, -1.8);
}

TEST(TurretEnvelope, PitchRangeInterpolatesOnRamp)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());
  // 傾斜区間の中点なので上限は 3.14 と 0.0 の中間
  EXPECT_NEAR(envelope.pitchRangeAt(0.275).max, 1.57, 1e-9);
}

TEST(TurretEnvelope, PitchRangeClampsOutsideDomain)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());
  EXPECT_DOUBLE_EQ(envelope.pitchRangeAt(-5.0).max, 3.14);
  EXPECT_DOUBLE_EQ(envelope.pitchRangeAt(5.0).max, 0.0);
}

// ---------------------------------------------------------------------------
// 掃過範囲: 通過する全角度で満たせる範囲
// ---------------------------------------------------------------------------

TEST(TurretEnvelope, SpanTakesMostRestrictiveLimit)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  // 自由領域から制限領域まで掃くなら、厳しい方（0.0）に従う
  const PitchRange span = envelope.pitchRangeOverSpan(0.0, 1.0);
  EXPECT_DOUBLE_EQ(span.max, 0.0);
  EXPECT_DOUBLE_EQ(span.min, -1.8);

  // 自由領域の中だけなら制限されない
  EXPECT_DOUBLE_EQ(envelope.pitchRangeOverSpan(-0.4, 0.2).max, 3.14);
}

TEST(TurretEnvelope, SpanIsSymmetric)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());
  EXPECT_DOUBLE_EQ(
    envelope.pitchRangeOverSpan(0.0, 1.0).max,
    envelope.pitchRangeOverSpan(1.0, 0.0).max);
}

// ---------------------------------------------------------------------------
// 逆引き: ピッチ -> ヨー範囲
// ---------------------------------------------------------------------------

TEST(TurretEnvelope, YawRangeIsFullWhenPitchIsLow)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  // ピッチ -1.0 はどこでも許されるので、定義域いっぱい動ける
  const YawRange range = envelope.yawRangeFor(-1.0, 0.0);
  EXPECT_NEAR(range.min, -0.48, 1e-9);
  EXPECT_NEAR(range.max, 2.20, 1e-9);
}

TEST(TurretEnvelope, YawRangeShrinksAsPitchRises)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  // 上限は 0.25->0.30 で 3.14->0.0 に落ちる。
  // ピッチ 1.57 はちょうど中点まで。
  const YawRange range = envelope.yawRangeFor(1.57, 0.0);
  EXPECT_NEAR(range.max, 0.275, 1e-6);

  // より高いピッチならもっと手前で止まる
  EXPECT_LT(envelope.yawRangeFor(2.5, 0.0).max, range.max);
  // より低いピッチならもっと先へ行ける
  EXPECT_GT(envelope.yawRangeFor(0.5, 0.0).max, range.max);
}

TEST(TurretEnvelope, YawRangeIsEmptyWhenPitchIsOutsideEnvelope)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  // 制限領域(yaw=1.0)に居て、ピッチが上限 0.0 を超えている
  const YawRange range = envelope.yawRangeFor(1.5, 1.0);
  EXPECT_DOUBLE_EQ(range.min, 1.0) << "ヨーは動かせない";
  EXPECT_DOUBLE_EQ(range.max, 1.0);
}

// ---------------------------------------------------------------------------
// apply: 総合
// ---------------------------------------------------------------------------

TEST(TurretEnvelope, AllowsFreeMotionInsideEnvelope)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  const EnvelopeResult result = envelope.apply(0.1, 2.0, 0.0, 2.0);
  EXPECT_DOUBLE_EQ(result.yaw, 0.1);
  EXPECT_DOUBLE_EQ(result.pitch, 2.0);
  EXPECT_FALSE(result.recovering);
}

TEST(TurretEnvelope, StopsYawWhereCurrentPitchIsNoLongerAllowed)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  // 砲身は yaw=0.0 / pitch=1.57。yaw=1.0 を要求しても、
  // そのピッチで入れるのは 0.275 まで。
  const EnvelopeResult result = envelope.apply(1.0, 1.57, 0.0, 1.57);
  EXPECT_NEAR(result.yaw, 0.275, 1e-6);
  EXPECT_FALSE(result.recovering);
}

TEST(TurretEnvelope, YawOpensUpAsPitchComesDown)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  const double high = envelope.apply(2.0, 1.57, 0.0, 1.57).yaw;
  const double mid = envelope.apply(2.0, 0.5, 0.0, 0.5).yaw;
  const double low = envelope.apply(2.0, -1.0, 0.0, -1.0).yaw;

  EXPECT_LT(high, mid) << "ピッチが下がるほど先へ進める";
  EXPECT_LT(mid, low);
  EXPECT_NEAR(low, 2.0, 1e-9) << "十分低ければ要求どおり";
}

TEST(TurretEnvelope, LimitsPitchOverTheSweptSpan)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  // 制限領域(yaw=1.0)から自由領域(yaw=0.0)へ戻る途中。
  // 砲身はまだ制限領域を通過するので、ピッチを上げてはいけない。
  const EnvelopeResult result = envelope.apply(0.0, 3.0, 1.0, -1.0);
  EXPECT_DOUBLE_EQ(result.pitch, 0.0) << "掃過範囲の最も厳しい上限に従う";
}

TEST(TurretEnvelope, RecoversWhenPitchIsOutsideEnvelope)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  // 手で動かされて、制限領域でピッチが上がっている状態
  const EnvelopeResult result = envelope.apply(2.0, 1.5, 1.0, 1.5);
  EXPECT_DOUBLE_EQ(result.yaw, 1.0) << "ヨーは固定";
  EXPECT_DOUBLE_EQ(result.pitch, 0.0) << "ピッチを許容範囲へ戻す";
  EXPECT_TRUE(result.recovering);
}

TEST(TurretEnvelope, AppliesMotorLimitsOnTopOfEnvelope)
{
  const TempYaml yaml(kRightLike);
  TurretEnvelope envelope = loadOrFail(yaml.path());
  MotorLimits motor;
  motor.yaw_min = -0.1;
  motor.yaw_max = 0.1;
  motor.pitch_min = -1.0;
  motor.pitch_max = 1.0;
  envelope.setMotorLimits(motor);

  const EnvelopeResult result = envelope.apply(2.0, 3.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(result.yaw, 0.1);
  EXPECT_DOUBLE_EQ(result.pitch, 1.0);
}

TEST(TurretEnvelope, EmptyEnvelopeAppliesOnlyMotorLimits)
{
  TurretEnvelope envelope;
  MotorLimits motor;
  motor.yaw_min = -1.0;
  motor.yaw_max = 1.0;
  motor.pitch_min = -1.0;
  motor.pitch_max = 1.0;
  envelope.setMotorLimits(motor);

  EXPECT_TRUE(envelope.empty());
  const EnvelopeResult result = envelope.apply(5.0, 5.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(result.yaw, 1.0);
  EXPECT_DOUBLE_EQ(result.pitch, 1.0);
}

TEST(TurretEnvelope, IsStateless)
{
  const TempYaml yaml(kRightLike);
  const TurretEnvelope envelope = loadOrFail(yaml.path());

  // 何度呼んでも、途中で別の入力を挟んでも、同じ入力なら同じ結果になる
  const EnvelopeResult first = envelope.apply(1.0, 1.57, 0.0, 1.57);
  envelope.apply(-0.4, -1.0, 2.0, 0.0);
  envelope.apply(2.0, 0.0, 1.0, 0.0);
  const EnvelopeResult again = envelope.apply(1.0, 1.57, 0.0, 1.57);

  EXPECT_DOUBLE_EQ(first.yaw, again.yaw);
  EXPECT_DOUBLE_EQ(first.pitch, again.pitch);
}

// ---------------------------------------------------------------------------
// 実際に配布する包絡線
// ---------------------------------------------------------------------------

TEST(TurretEnvelope, ShippedEnvelopesLoad)
{
  for (const char * side : {"left", "right"}) {
    const std::string path = std::string(CORE_SHOOTER_CONFIG_DIR) +
      "/turret_envelope_" + side + ".yaml";
    if (!std::filesystem::exists(path)) {
      GTEST_SKIP() << "config not installed: " << path;
    }
    TurretEnvelope envelope;
    std::string reason;
    ASSERT_TRUE(TurretEnvelope::load(path, envelope, reason)) << side << ": " << reason;
    EXPECT_GE(envelope.size(), 2u) << side;
    EXPECT_LT(envelope.yawMin(), envelope.yawMax()) << side;
  }
}

TEST(TurretEnvelope, ShippedRightEnvelopeBlocksRaisingBeyondTheRamp)
{
  const std::string path = std::string(CORE_SHOOTER_CONFIG_DIR) + "/turret_envelope_right.yaml";
  if (!std::filesystem::exists(path)) {
    GTEST_SKIP() << "config not installed";
  }
  TurretEnvelope envelope;
  std::string reason;
  ASSERT_TRUE(TurretEnvelope::load(path, envelope, reason)) << reason;

  // 旧 zone 設定の意図: ヨー 0.25 を超えたら水平より上げない
  EXPECT_GT(envelope.pitchRangeAt(0.0).max, 1.0);
  EXPECT_DOUBLE_EQ(envelope.pitchRangeAt(1.0).max, 0.0);
}
