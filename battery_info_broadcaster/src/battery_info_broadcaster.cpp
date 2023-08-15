#include "battery_info_broadcaster/battery_info_broadcaster.hpp"

#include <memory>
#include <string>

namespace battery_info_broadcaster
{
controller_interface::CallbackReturn BatteryInfoBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  // Fill interfaces
  // TODO: Parametrize battery interface names
  battery_fields_["low_speed_controller/battery_voltage"] = -1;
  battery_fields_["low_speed_controller/battery_current"] = -1;
  battery_fields_["low_speed_controller/battery_soc"] = -1;
  battery_fields_["low_speed_controller/battery_status"] = -1;

  for(int i = 0; i < 8; i++){
    std::string cell_state_name = "low_speed_controller/cell_" + std::to_string(i) + "_voltage";
    battery_fields_[cell_state_name] = -1;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BatteryInfoBroadcaster::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration BatteryInfoBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  std::map<std::string, double>::const_iterator it = battery_fields_.begin();
  while (it != battery_fields_.end())
  {
    state_interfaces_config.names.push_back(it->first.c_str());
    ++it;
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn BatteryInfoBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  battery_info_publisher_ = get_node()->create_publisher<robot_msgs::msg::BatteryInfo>(
    params_.battery_info_topic, rclcpp::SystemDefaultsQoS());
  battery_info_realtime_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<robot_msgs::msg::BatteryInfo>>(battery_info_publisher_);

  battery_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::BatteryState>(
    params_.battery_state_topic, rclcpp::SystemDefaultsQoS());
  battery_state_realtime_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>>(battery_state_publisher_);

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryInfoBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryInfoBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type BatteryInfoBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

}  // namespace battery_info_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  battery_info_broadcaster::BatteryInfoBroadcaster, controller_interface::ControllerInterface)
