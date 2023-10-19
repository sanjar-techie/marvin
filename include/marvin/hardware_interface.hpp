// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MARVIN__DIFFBOT_SYSTEM_HPP_
#define MARVIN__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "marvin/visibility_control.h"

#include "marvin/serial_comms.hpp"
// #include "marvin/wheel.hpp"

namespace marvin
{
class MarvinHardware : public hardware_interface::SystemInterface
{

struct Config 
{
  int use_MDUI;
  int nIDPC;
  int nIDMDUI;
  int nIDMDT;
  int nBaudrate;
  int nDiameter;
  double wheel_radius;
  double nWheelLength;
  int nRMID;
  int nGearRatio;
  int reverse_direction;
  int motor_position_type;
  int encoder_PPR;
  int nMaxRPM;
  int position_proportion_gain;
  int speed_proportion_gain;
  int integral_gain;
  int nSlowstart;
  int nSlowdown;
  int motor_pole;
  // int motor_count;
  // double motor_count_per_degree;

  uint16_t sSetDia;
  uint16_t sSetWheelLen;
  uint16_t sSetGear;
}

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MarvinHardware);

  MARVIN_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  MARVIN_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MARVIN_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MARVIN_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MARVIN_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  MARVIN_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MARVIN_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  MARVIN_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MARVIN_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  SerialPort comms_; // to be changed
  Config cfg_;
//   Wheel wheel_l_;
//   Wheel wheel_r_;
};

}  // namespace marvin

#endif  // MARVIN__DIFFBOT_SYSTEM_HPP_