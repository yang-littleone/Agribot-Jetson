#include <gtest/gtest.h>

#include "turn_on_32chassis/wheel_trust_monitor.hpp"

using turn_on_32chassis::WheelTrustMonitor;
using turn_on_32chassis::WheelTrustState;

TEST(WheelTrustMonitor, KeepsConsistentWheelTrusted)
{
  WheelTrustMonitor monitor;
  monitor.update(0.0, 0.005, 0.01, 0.20);
  monitor.update(1.0, 0.005, 0.01, 0.20);
  EXPECT_EQ(monitor.state(), WheelTrustState::TRUSTED);
  EXPECT_DOUBLE_EQ(monitor.variance_scale(1.0), 1.0);
}

TEST(WheelTrustMonitor, RejectsPersistentSlip)
{
  WheelTrustMonitor monitor;
  monitor.update(0.0, 0.10, 0.01, 0.20);
  EXPECT_EQ(monitor.state(), WheelTrustState::SUSPECT);
  EXPECT_GT(monitor.variance_scale(0.1), 1.0);
  EXPECT_DOUBLE_EQ(monitor.prediction_weight(0.1), 0.0);

  monitor.update(0.6, 0.10, 0.01, 0.20);
  EXPECT_EQ(monitor.state(), WheelTrustState::REJECTED);
  EXPECT_TRUE(monitor.reject_wheel());
  EXPECT_DOUBLE_EQ(monitor.prediction_weight(0.6), 0.0);
}

TEST(WheelTrustMonitor, RejectsPartialTranslationSlip)
{
  WheelTrustMonitor monitor;
  monitor.update(0.0, 0.04, 0.01, 0.20);
  monitor.update(0.6, 0.04, 0.01, 0.20);
  EXPECT_EQ(monitor.state(), WheelTrustState::REJECTED);
}

TEST(WheelTrustMonitor, RejectsYawSlipDuringTurning)
{
  WheelTrustMonitor monitor;
  monitor.update(0.0, 0.005, 0.20, 0.20);
  monitor.update(0.6, 0.005, 0.20, 0.20);
  EXPECT_EQ(monitor.state(), WheelTrustState::REJECTED);
}

TEST(WheelTrustMonitor, RejectsZeroWheelWhenLioObservesExternalMotion)
{
  WheelTrustMonitor monitor;
  monitor.update(0.0, 0.10, 0.01, 0.10);
  monitor.update(0.6, 0.10, 0.01, 0.10);
  EXPECT_EQ(monitor.state(), WheelTrustState::REJECTED);
}

TEST(WheelTrustMonitor, IgnoresResidualWhenNothingMoved)
{
  WheelTrustMonitor monitor;
  monitor.update(0.0, 0.10, 0.20, 0.001);
  EXPECT_EQ(monitor.state(), WheelTrustState::TRUSTED);
}

TEST(WheelTrustMonitor, RecoversOnlyAfterStableAgreement)
{
  WheelTrustMonitor monitor;
  monitor.update(0.0, 0.10, 0.01, 0.20);
  monitor.update(0.6, 0.10, 0.01, 0.20);
  ASSERT_EQ(monitor.state(), WheelTrustState::REJECTED);

  monitor.update(0.7, 0.005, 0.01, 0.20);
  EXPECT_EQ(monitor.state(), WheelTrustState::REJECTED);

  monitor.update(1.8, 0.005, 0.01, 0.20);
  EXPECT_EQ(monitor.state(), WheelTrustState::RECOVERING);
  EXPECT_GT(monitor.variance_scale(2.3), 1.0);
  EXPECT_NEAR(monitor.prediction_weight(2.3), 0.5, 1.0e-9);

  monitor.update(2.9, 0.005, 0.01, 0.20);
  EXPECT_EQ(monitor.state(), WheelTrustState::TRUSTED);
  EXPECT_DOUBLE_EQ(monitor.variance_scale(2.9), 1.0);
  EXPECT_DOUBLE_EQ(monitor.prediction_weight(2.9), 1.0);
}

TEST(WheelTrustMonitor, StartsRejectedUntilAgreementIsProven)
{
  WheelTrustMonitor monitor;
  monitor.start_rejected(0.0);
  EXPECT_TRUE(monitor.reject_wheel());

  monitor.update(0.1, 0.005, 0.01, 0.20);
  monitor.update(1.2, 0.005, 0.01, 0.20);
  EXPECT_EQ(monitor.state(), WheelTrustState::RECOVERING);
  EXPECT_GT(monitor.variance_scale(1.2), 1.0);
}
