// core_hardware ノードが出す /hardware/motor_status・/hardware/link_status の変換 (core_hardware.hpp) の単体テスト
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "core_hardware/core_hardware.hpp"

namespace {

using core_hardware::HardwareSnapshot;
namespace md = core_hardware::motor_diag;

TEST(MotorStatusArray, ListsAllMotorsAndBusesInOrder) {
  const auto msg = core_hardware::make_motor_status_array(HardwareSnapshot{});

  ASSERT_EQ(msg.motors.size(), 15U);
  for (std::size_t i = 0; i < msg.motors.size(); ++i) {
    const auto& motor = msg.motors[i];
    EXPECT_EQ(motor.id, static_cast<uint8_t>(i));
    EXPECT_FALSE(motor.supported) << "id=" << i;
    EXPECT_TRUE(std::isnan(motor.temp)) << "id=" << i;
    EXPECT_TRUE(std::isnan(motor.temp_rotor)) << "id=" << i;
    EXPECT_TRUE(std::isnan(motor.voltage)) << "id=" << i;
  }
  ASSERT_EQ(msg.can_buses.size(), 2U);
  EXPECT_EQ(msg.can_buses[0].name, "can3");
  EXPECT_EQ(msg.can_buses[1].name, "can2");
  EXPECT_FALSE(msg.can_buses[0].valid);
  EXPECT_FALSE(msg.can_buses[1].valid);
}

TEST(MotorStatusArray, ConvertsSupportedMotors) {
  HardwareSnapshot state;

  md::MotorDiag damiao;
  damiao.supported = true;
  damiao.connected = true;
  damiao.enabled = true;
  damiao.fault = true;
  damiao.error_code = 0x0BU;
  damiao.temp = 45;
  damiao.temp_rotor = -3;
  state.motor_diag[2] = md::pack_motor_diag(damiao);
  state.motor_voltage[2] = md::kVoltageInvalid;

  md::MotorDiag feetech;
  feetech.supported = true;
  feetech.connected = true;
  feetech.enabled = false;
  feetech.temp = 38;
  state.motor_diag[9] = md::pack_motor_diag(feetech);
  state.motor_voltage[9] = 1210U;

  const auto msg = core_hardware::make_motor_status_array(state);

  const auto& m2 = msg.motors[2];
  EXPECT_EQ(m2.id, 2U);
  EXPECT_TRUE(m2.supported);
  EXPECT_TRUE(m2.connected);
  EXPECT_TRUE(m2.enabled);
  EXPECT_TRUE(m2.fault);
  EXPECT_EQ(m2.error_code, 0x0BU);
  EXPECT_FLOAT_EQ(m2.temp, 45.0f);
  EXPECT_FLOAT_EQ(m2.temp_rotor, -3.0f);
  EXPECT_TRUE(std::isnan(m2.voltage));

  const auto& m9 = msg.motors[9];
  EXPECT_EQ(m9.id, 9U);
  EXPECT_TRUE(m9.supported);
  EXPECT_TRUE(m9.connected);
  EXPECT_FALSE(m9.enabled);
  EXPECT_FALSE(m9.fault);
  EXPECT_EQ(m9.error_code, 0U);
  EXPECT_FLOAT_EQ(m9.temp, 38.0f);
  EXPECT_TRUE(std::isnan(m9.temp_rotor));
  EXPECT_FLOAT_EQ(m9.voltage, 12.1f);
}

TEST(MotorStatusArray, MasksValuesOfUnsupportedMotor) {
  HardwareSnapshot state;
  // supported のビットが無ければ、他のビットや電圧が入っていても無効値として扱う
  state.motor_diag[0] = md::kMotorConnectedBit | md::kMotorEnabledBit | md::kMotorFaultBit |
                        (0x12U << md::kMotorErrorCodeShift) | (40U << md::kMotorTempShift);
  state.motor_voltage[0] = 2400U;

  const auto msg = core_hardware::make_motor_status_array(state);
  const auto& motor = msg.motors[0];
  EXPECT_FALSE(motor.supported);
  EXPECT_FALSE(motor.connected);
  EXPECT_FALSE(motor.enabled);
  EXPECT_FALSE(motor.fault);
  EXPECT_EQ(motor.error_code, 0U);
  EXPECT_TRUE(std::isnan(motor.temp));
  EXPECT_TRUE(std::isnan(motor.temp_rotor));
  EXPECT_TRUE(std::isnan(motor.voltage));
}

TEST(MotorStatusArray, ConvertsBusDiag) {
  HardwareSnapshot state;
  md::BusDiag can3;
  can3.valid = true;
  can3.error_passive = true;
  can3.tx_error_count = 130U;
  can3.rx_error_count = 5U;
  state.bus_diag[0] = md::pack_bus_diag(can3);
  // valid でなければ bus_off のビットが立っていても無効値として扱う
  state.bus_diag[1] = md::kBusOffBit | (7U << md::kBusTxErrorCountShift);

  const auto msg = core_hardware::make_motor_status_array(state);

  const auto& bus3 = msg.can_buses[0];
  EXPECT_EQ(bus3.name, "can3");
  EXPECT_TRUE(bus3.valid);
  EXPECT_FALSE(bus3.bus_off);
  EXPECT_TRUE(bus3.error_passive);
  EXPECT_EQ(bus3.tx_error_count, 130U);
  EXPECT_EQ(bus3.rx_error_count, 5U);

  const auto& bus2 = msg.can_buses[1];
  EXPECT_EQ(bus2.name, "can2");
  EXPECT_FALSE(bus2.valid);
  EXPECT_FALSE(bus2.bus_off);
  EXPECT_FALSE(bus2.error_passive);
  EXPECT_EQ(bus2.tx_error_count, 0U);
  EXPECT_EQ(bus2.rx_error_count, 0U);
}

HardwareSnapshot make_connected_state() {
  HardwareSnapshot state;
  state.ethercat_connected = {1U};
  state.last_ethercat_rx_ms = {5000U};
  state.system_state = {1U, 1U};
  return state;
}

TEST(LinkStatus, AllConnected) {
  const auto msg = core_hardware::make_link_status(true, make_connected_state(), 5100);
  EXPECT_TRUE(msg.daemon_connected);
  EXPECT_TRUE(msg.ethercat_connected);
  EXPECT_FLOAT_EQ(msg.ethercat_rx_age_ms, 100.0f);
  EXPECT_TRUE(msg.bottom_connected);
}

TEST(LinkStatus, BottomDisconnected) {
  HardwareSnapshot state = make_connected_state();
  state.system_state[1] = 0U;
  const auto msg = core_hardware::make_link_status(true, state, 5100);
  EXPECT_TRUE(msg.ethercat_connected);
  EXPECT_FALSE(msg.bottom_connected);
}

TEST(LinkStatus, EthercatStaleHidesBottom) {
  const auto msg = core_hardware::make_link_status(true, make_connected_state(), 5000 + core_hardware::kEthercatRxTimeoutMs + 1);
  EXPECT_TRUE(msg.daemon_connected);
  EXPECT_FALSE(msg.ethercat_connected);
  EXPECT_FLOAT_EQ(msg.ethercat_rx_age_ms, static_cast<float>(core_hardware::kEthercatRxTimeoutMs + 1));
  EXPECT_FALSE(msg.bottom_connected);
}

TEST(LinkStatus, EmptySnapshotFromDaemon) {
  // EtherCAT が落ちたとき daemon は空のスナップショットを送る
  const auto msg = core_hardware::make_link_status(true, HardwareSnapshot{}, 5000);
  EXPECT_TRUE(msg.daemon_connected);
  EXPECT_FALSE(msg.ethercat_connected);
  EXPECT_TRUE(std::isnan(msg.ethercat_rx_age_ms));
  EXPECT_FALSE(msg.bottom_connected);
}

TEST(LinkStatus, DaemonDisconnectedHidesEverything) {
  const auto msg = core_hardware::make_link_status(false, make_connected_state(), 5100);
  EXPECT_FALSE(msg.daemon_connected);
  EXPECT_FALSE(msg.ethercat_connected);
  EXPECT_TRUE(std::isnan(msg.ethercat_rx_age_ms));
  EXPECT_FALSE(msg.bottom_connected);
}

}  // namespace
