#include "standart_message_broadcaster/standart_message_broadcaster.hpp"

#include <memory>
#include <string>

namespace standart_message_broadcaster
{
controller_interface::CallbackReturn StandartMessageBroadcaster::on_init()
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

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration StandartMessageBroadcaster::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration StandartMessageBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  for (const auto & interface : params_.interfaces)
  {
    state_interfaces_config.names.push_back(interface);
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn StandartMessageBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  // Check parameters
  int interface_size = params_.interfaces.size();
  int topic_size = params_.topics.size();
  int type_size = params_.types.size();

  if(interface_size == 0 || topic_size == 0 || type_size == 0){
    RCLCPP_ERROR(get_node()->get_logger(),
      "Interface, topic and type parameters should have at least one element!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if(topic_size != interface_size || type_size != interface_size){
    RCLCPP_ERROR(get_node()->get_logger(),
      "Interface, topic and type parameters should have same size!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Create publishers
  for(int i = 0; i < interface_size; i++){
    publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      params_.topics[i], rclcpp::SystemDefaultsQoS());

    realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(publisher_);

    realtime_publisher_->msg_.data = false;

    break; // TODO: Create vector of publishers
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StandartMessageBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StandartMessageBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type StandartMessageBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (const auto & state_interface : state_interfaces_)
  {
    if(state_interface.get_name() != params_.interfaces[0]) continue;

    if(realtime_publisher_ && realtime_publisher_->trylock())
    {
      auto & message = realtime_publisher_->msg_;
      message.data = (state_interface.get_value() == 1.0) ? true : false;
      realtime_publisher_->unlockAndPublish();

      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      RCLCPP_INFO(get_node()->get_logger(), "Motor status: %s",
        message.data ? "OK" : "Not-OK");
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace standart_message_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  standart_message_broadcaster::StandartMessageBroadcaster, controller_interface::ControllerInterface)
