#ifndef BATTERY_INFO_BROADCASTER__BATTERY_INFO_BROADCASTER_HPP_
#define BATTERY_INFO_BROADCASTER__BATTERY_INFO_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <algorithm>

#include "controller_interface/controller_interface.hpp"
#include "battery_info_broadcaster/visibility_control.h"
#include "battery_info_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "sensor_msgs/msg/battery_state.hpp"
#include "robot_msgs/msg/battery_info.hpp"

namespace battery_info_broadcaster
{
class BatteryInfoBroadcaster : public controller_interface::ControllerInterface
{
public:
  BATTERY_INFO_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  BATTERY_INFO_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  BATTERY_INFO_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

  BATTERY_INFO_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  BATTERY_INFO_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  BATTERY_INFO_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  BATTERY_INFO_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::map<std::string, double> battery_fields_;

  std::shared_ptr<rclcpp::Publisher<robot_msgs::msg::BatteryInfo>> battery_info_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<robot_msgs::msg::BatteryInfo>> battery_info_realtime_publisher_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> battery_state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>> battery_state_realtime_publisher_;
};

}  // namespace battery_info_broadcaster

#endif  // BATTERY_INFO_BROADCASTER__BATTERY_INFO_BROADCASTER_HPP_
