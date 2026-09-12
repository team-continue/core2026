#ifndef CORE_SHOOTER__TEST_MODE_GATE_HPP_
#define CORE_SHOOTER__TEST_MODE_GATE_HPP_

namespace core_shooter
{

/// test_mode の実効値を保持する。
///
/// `/test_mode` トピックを 1 度でも受信していればその値を、
/// 未受信の間はパラメータ `enable_test_mode` の既定値を採用する。
class TestModeGate
{
public:
  TestModeGate() = default;

  /// パラメータ由来の既定値を設定する（トピック未受信時に使われる）。
  void setDefault(bool default_value)
  {
    default_value_ = default_value;
  }

  /// トピック値を反映する。実効値が変化した場合のみ true を返す。
  bool update(bool topic_value)
  {
    const bool previous_effective = enabled();
    topic_value_ = topic_value;
    has_topic_value_ = true;
    return enabled() != previous_effective;
  }

  /// 現在の実効値。
  bool enabled() const
  {
    return has_topic_value_ ? topic_value_ : default_value_;
  }

  /// パラメータ由来の既定値（ログ出力用）。
  bool defaultValue() const
  {
    return default_value_;
  }

private:
  bool default_value_ = false;
  bool topic_value_ = false;
  bool has_topic_value_ = false;
};

}  // namespace core_shooter

#endif  // CORE_SHOOTER__TEST_MODE_GATE_HPP_
