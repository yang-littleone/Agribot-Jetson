#ifndef TURN_ON_32CHASSIS__WHEEL_TRUST_MONITOR_HPP_
#define TURN_ON_32CHASSIS__WHEEL_TRUST_MONITOR_HPP_

#include <algorithm>
#include <cmath>

namespace turn_on_32chassis
{

enum class WheelTrustState
{
  TRUSTED,
  SUSPECT,
  REJECTED,
  RECOVERING,
};

struct WheelTrustConfig
{
  double translation_threshold{0.03};
  double yaw_threshold{0.12};
  double recovery_translation_threshold{0.015};
  double recovery_yaw_threshold{0.06};
  double minimum_motion{0.015};
  double confirmation_time{0.5};
  double recovery_time{1.0};
  double recovery_ramp_time{1.0};
  double suspect_variance_scale{25.0};
  double rejected_variance_scale{1.0e6};
};

class WheelTrustMonitor
{
public:
  explicit WheelTrustMonitor(const WheelTrustConfig & config = WheelTrustConfig())
  : config_(config)
  {
  }

  void reset()
  {
    state_ = WheelTrustState::TRUSTED;
    state_start_time_ = 0.0;
  }

  void start_rejected(double now_seconds)
  {
    recovery_candidate_ = false;
    set_state(WheelTrustState::REJECTED, now_seconds);
  }

  void update(
    double now_seconds, double translation_error, double yaw_error,
    double observed_motion)
  {
    const bool motion_is_observable = observed_motion >= config_.minimum_motion;
    const bool inconsistent =
      motion_is_observable &&
      (translation_error > config_.translation_threshold ||
      yaw_error > config_.yaw_threshold);
    const bool consistent =
      translation_error < config_.recovery_translation_threshold &&
      yaw_error < config_.recovery_yaw_threshold;

    switch (state_) {
      case WheelTrustState::TRUSTED:
        if (inconsistent) {
          set_state(WheelTrustState::SUSPECT, now_seconds);
        }
        break;
      case WheelTrustState::SUSPECT:
        if (!inconsistent) {
          set_state(WheelTrustState::TRUSTED, now_seconds);
        } else if (now_seconds - state_start_time_ >= config_.confirmation_time) {
          set_state(WheelTrustState::REJECTED, now_seconds);
        }
        break;
      case WheelTrustState::REJECTED:
        if (consistent) {
          if (!recovery_candidate_) {
            recovery_candidate_ = true;
            recovery_candidate_start_time_ = now_seconds;
          } else if (
            now_seconds - recovery_candidate_start_time_ >=
            config_.recovery_time)
          {
            recovery_candidate_ = false;
            set_state(WheelTrustState::RECOVERING, now_seconds);
          }
        } else {
          recovery_candidate_ = false;
        }
        break;
      case WheelTrustState::RECOVERING:
        if (inconsistent || !consistent) {
          set_state(WheelTrustState::REJECTED, now_seconds);
        } else if (
          now_seconds - state_start_time_ >= config_.recovery_ramp_time)
        {
          set_state(WheelTrustState::TRUSTED, now_seconds);
        }
        break;
    }
  }

  WheelTrustState state() const
  {
    return state_;
  }

  bool reject_wheel() const
  {
    return state_ == WheelTrustState::REJECTED;
  }

  double prediction_weight(double now_seconds) const
  {
    if (state_ == WheelTrustState::TRUSTED) {
      return 1.0;
    }
    if (state_ != WheelTrustState::RECOVERING) {
      return 0.0;
    }
    return std::clamp(
      (now_seconds - state_start_time_) /
      std::max(config_.recovery_ramp_time, 1.0e-6),
      0.0, 1.0);
  }

  double variance_scale(double now_seconds) const
  {
    if (state_ == WheelTrustState::TRUSTED) {
      return 1.0;
    }
    if (state_ == WheelTrustState::REJECTED) {
      return config_.rejected_variance_scale;
    }

    const double duration =
      state_ == WheelTrustState::SUSPECT ?
      std::max(config_.confirmation_time, 1.0e-6) :
      std::max(config_.recovery_ramp_time, 1.0e-6);
    const double progress =
      std::clamp((now_seconds - state_start_time_) / duration, 0.0, 1.0);

    if (state_ == WheelTrustState::SUSPECT) {
      return config_.suspect_variance_scale *
             std::pow(
        config_.rejected_variance_scale / config_.suspect_variance_scale,
        progress);
    }
    return config_.rejected_variance_scale *
           std::pow(1.0 / config_.rejected_variance_scale, progress);
  }

private:
  void set_state(WheelTrustState state, double now_seconds)
  {
    state_ = state;
    state_start_time_ = now_seconds;
    if (state != WheelTrustState::REJECTED) {
      recovery_candidate_ = false;
    }
  }

  WheelTrustConfig config_;
  WheelTrustState state_{WheelTrustState::TRUSTED};
  double state_start_time_{0.0};
  bool recovery_candidate_{false};
  double recovery_candidate_start_time_{0.0};
};

}  // namespace turn_on_32chassis

#endif  // TURN_ON_32CHASSIS__WHEEL_TRUST_MONITOR_HPP_
