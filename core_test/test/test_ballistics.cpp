// core_shooter::BallisticTable の単体テスト。
//
// 曲射弾道の実測テーブル（config/ballistics_*.yaml）の読み込みと、
// 距離から射角補正への換算を検証する。ROS の起動は不要。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "core_shooter/ballistics.hpp"

using core_shooter::BallisticTable;
using core_shooter::DirectionAngles;

namespace
{

/// テスト用の YAML を一時ファイルへ書き出し、デストラクタで消す。
class TempYaml
{
public:
  explicit TempYaml(const std::string & body)
  {
    static int counter = 0;
    path_ = (std::filesystem::temp_directory_path() /
      ("core_test_ballistics_" + std::to_string(++counter) + ".yaml")).string();
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

/// 落下のみのテーブル。2.0m で 0.2m、3.0m で 0.6m 落ちる。
constexpr const char * kDropTable =
  "ballistics:\n"
  "  2.0: {y: 0.0, z: -0.2}\n"
  "  3.0: {y: 0.0, z: -0.6}\n";

BallisticTable loadOrFail(const std::string & path)
{
  BallisticTable table;
  std::string reason;
  EXPECT_TRUE(BallisticTable::load(path, table, reason)) << reason;
  return table;
}

}  // namespace

// ---------------------------------------------------------------------------
// 未設定のテーブル
// ---------------------------------------------------------------------------

TEST(BallisticTable, EmptyTableMeansStraightTrajectory)
{
  const BallisticTable table;
  EXPECT_TRUE(table.empty());

  const DirectionAngles correction = table.correction(3.0);
  EXPECT_DOUBLE_EQ(correction.yaw_rad, 0.0);
  EXPECT_DOUBLE_EQ(correction.pitch_rad, 0.0);
  // 覆うべき範囲という概念がないので、はみ出し量も 0。
  EXPECT_DOUBLE_EQ(table.rangeError(100.0), 0.0);
}

// ---------------------------------------------------------------------------
// 読み込み
// ---------------------------------------------------------------------------

TEST(BallisticTable, LoadsSamplesSortedByRange)
{
  // わざと降順で書いても、距離の昇順に並べ替えられる。
  const TempYaml yaml(
    "ballistics:\n"
    "  3.0: {y: 0.0, z: -0.6}\n"
    "  1.0: {y: 0.0, z: 0.0}\n"
    "  2.0: {y: 0.0, z: -0.2}\n");

  const BallisticTable table = loadOrFail(yaml.path());
  EXPECT_EQ(table.size(), 3u);
  EXPECT_DOUBLE_EQ(table.minRange(), 1.0);
  EXPECT_DOUBLE_EQ(table.maxRange(), 3.0);
}

TEST(BallisticTable, RejectsBrokenConfigurations)
{
  struct Case
  {
    const char * name;
    const char * body;
  };
  const Case cases[] = {
    {"ballistics キーがない", "other:\n  1.0: {y: 0.0, z: 0.0}\n"},
    {"エントリがマップでない", "ballistics:\n  1.0: 5\n"},
    {"z が欠けている", "ballistics:\n  1.0: {y: 0.0}\n"},
    {"y が欠けている", "ballistics:\n  1.0: {z: 0.0}\n"},
    {"距離が負", "ballistics:\n  -1.0: {y: 0.0, z: 0.0}\n"},
    {"距離が 0", "ballistics:\n  0.0: {y: 0.0, z: 0.0}\n"},
    {"テーブルが空", "ballistics: {}\n"},
  };

  for (const Case & c : cases) {
    const TempYaml yaml(c.body);
    BallisticTable table;
    std::string reason;
    EXPECT_FALSE(BallisticTable::load(yaml.path(), table, reason)) << c.name;
    EXPECT_FALSE(reason.empty()) << c.name;
  }
}

TEST(BallisticTable, RejectsMissingFile)
{
  BallisticTable table;
  std::string reason;
  EXPECT_FALSE(BallisticTable::load("/nonexistent/ballistics.yaml", table, reason));
  EXPECT_FALSE(reason.empty());
}

// ---------------------------------------------------------------------------
// 射角補正への換算
// ---------------------------------------------------------------------------

TEST(BallisticTable, DropIsCompensatedByAimingUp)
{
  const TempYaml yaml(kDropTable);
  const BallisticTable table = loadOrFail(yaml.path());

  // 2.0m で 0.2m 落ちるなら、その分だけ上へ向ける。
  const DirectionAngles correction = table.correction(2.0);
  EXPECT_NEAR(correction.pitch_rad, std::atan2(0.2, 2.0), 1e-9);
  EXPECT_NEAR(correction.yaw_rad, 0.0, 1e-9);
  EXPECT_GT(correction.pitch_rad, 0.0) << "落下は上向き補正になるはず";
}

TEST(BallisticTable, LateralDriftIsCompensatedInOppositeDirection)
{
  // y は左が正。左へ 0.1m 逸れるなら、右（負）へ振る。
  const TempYaml yaml(
    "ballistics:\n"
    "  2.0: {y: 0.1, z: 0.0}\n"
    "  3.0: {y: 0.3, z: 0.0}\n");
  const BallisticTable table = loadOrFail(yaml.path());

  const DirectionAngles correction = table.correction(2.0);
  EXPECT_NEAR(correction.yaw_rad, std::atan2(-0.1, 2.0), 1e-9);
  EXPECT_LT(correction.yaw_rad, 0.0) << "左へ逸れる弾は右向き補正になるはず";
}

TEST(BallisticTable, InterpolatesBetweenSamples)
{
  const TempYaml yaml(kDropTable);
  const BallisticTable table = loadOrFail(yaml.path());

  // 2.5m は 2.0m と 3.0m の補正角のちょうど中間。
  const double lower = std::atan2(0.2, 2.0);
  const double upper = std::atan2(0.6, 3.0);
  EXPECT_NEAR(table.correction(2.5).pitch_rad, (lower + upper) / 2.0, 1e-9);

  // 端点そのものはサンプル値と一致する。
  EXPECT_NEAR(table.correction(2.0).pitch_rad, lower, 1e-9);
  EXPECT_NEAR(table.correction(3.0).pitch_rad, upper, 1e-9);
}

TEST(BallisticTable, OutsideTableMeansNoCorrection)
{
  const TempYaml yaml(kDropTable);
  const BallisticTable table = loadOrFail(yaml.path());

  // テーブルが覆っていない距離では補正せず、的の方向へそのまま向ける。
  EXPECT_DOUBLE_EQ(table.correction(1.0).pitch_rad, 0.0);
  EXPECT_DOUBLE_EQ(table.correction(1.0).yaw_rad, 0.0);
  EXPECT_DOUBLE_EQ(table.correction(4.0).pitch_rad, 0.0);
  EXPECT_DOUBLE_EQ(table.correction(4.0).yaw_rad, 0.0);
}

// ---------------------------------------------------------------------------
// テーブル範囲からのはみ出し量
// ---------------------------------------------------------------------------

TEST(BallisticTable, RangeErrorIsZeroInsideAndGapOutside)
{
  const TempYaml yaml(kDropTable);
  const BallisticTable table = loadOrFail(yaml.path());

  EXPECT_DOUBLE_EQ(table.rangeError(2.0), 0.0);
  EXPECT_DOUBLE_EQ(table.rangeError(2.5), 0.0);
  EXPECT_DOUBLE_EQ(table.rangeError(3.0), 0.0);

  EXPECT_NEAR(table.rangeError(1.4), 0.6, 1e-9);
  EXPECT_NEAR(table.rangeError(3.7), 0.7, 1e-9);
}

// ---------------------------------------------------------------------------
// 実際に配布する設定ファイル
// ---------------------------------------------------------------------------

TEST(BallisticTable, ShippedTablesCoverHalfToSixMetres)
{
  // 500mm 刻みで 0.5m〜6.0m を覆っていること。左右とも同じ構成。
  for (const char * side : {"left", "right"}) {
    const std::string path = std::string(CORE_SHOOTER_CONFIG_DIR) +
      "/ballistics_" + side + ".yaml";
    if (!std::filesystem::exists(path)) {
      GTEST_SKIP() << "config not installed: " << path;
    }

    BallisticTable table;
    std::string reason;
    ASSERT_TRUE(BallisticTable::load(path, table, reason)) << side << ": " << reason;
    EXPECT_EQ(table.size(), 12u) << side;
    EXPECT_DOUBLE_EQ(table.minRange(), 0.5) << side;
    EXPECT_DOUBLE_EQ(table.maxRange(), 6.0) << side;

    // 実測前なので全て 0 = 直線弾道。実測値を入れたらこの期待値は変わる。
    EXPECT_DOUBLE_EQ(table.correction(3.0).pitch_rad, 0.0) << side;
    EXPECT_DOUBLE_EQ(table.correction(3.0).yaw_rad, 0.0) << side;
  }
}
