// 速度指令を、受けてから無駄時間だけたってから効かせる列。駆動の無駄時間（指令を受けてから応え始めるまで）
// を表す。ROS と Gazebo に依存しないので、プラグインと分けてテストする。

#ifndef CORE_GAZEBO__COMMAND_DELAY_HPP_
#define CORE_GAZEBO__COMMAND_DELAY_HPP_

#include <algorithm>
#include <cstddef>
#include <deque>
#include <utility>

namespace core_gazebo
{
template<typename Command>
class DelayedCommand
{
public:
  // dead_time_s が負なら 0 とする
  explicit DelayedCommand(const double dead_time_s = 0.0)
  : dead_time_s_(std::max(dead_time_s, 0.0)) {}

  // received_s（増える順）に受けた指令を足す。received_s + 無駄時間 から効く
  void push(const double received_s, const Command & command)
  {
    queue_.emplace_back(received_s + dead_time_s_, command);
  }

  // now_s までに効き始めた指令のうち最後のものを今の指令にして返す。まだなければ前の指令（初めは既定値）
  const Command & update(const double now_s)
  {
    while (!queue_.empty() && queue_.front().first <= now_s) {
      current_ = queue_.front().second;
      queue_.pop_front();
    }
    return current_;
  }

  double deadTime() const {return dead_time_s_;}
  // まだ効いていない指令の数
  std::size_t pending() const {return queue_.size();}

private:
  double dead_time_s_;
  std::deque<std::pair<double, Command>> queue_;
  Command current_{};
};
}  // namespace core_gazebo

#endif  // CORE_GAZEBO__COMMAND_DELAY_HPP_
