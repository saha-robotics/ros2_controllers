#ifndef STANDART_MESSAGE_BROADCASTER__STANDART_MESSAGE_BROADCASTER_HPP_
#define STANDART_MESSAGE_BROADCASTER__STANDART_MESSAGE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "standart_message_broadcaster/visibility_control.h"
#include "standart_message_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "std_msgs/msg/bool.hpp"

namespace standart_message_broadcaster
{
class StandartMessageBroadcaster : public controller_interface::ControllerInterface
{
public:
  STANDART_MESSAGE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  STANDART_MESSAGE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  STANDART_MESSAGE_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

  STANDART_MESSAGE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  STANDART_MESSAGE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  STANDART_MESSAGE_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  STANDART_MESSAGE_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>> realtime_publisher_;
};

}  // namespace standart_message_broadcaster

#endif  // STANDART_MESSAGE_BROADCASTER__STANDART_MESSAGE_BROADCASTER_HPP_
