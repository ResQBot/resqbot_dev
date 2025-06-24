#ifndef LOTTI_DRIVE_CONTROLLER__DRIVE_CONTROLLER_HPP_
#define LOTTI_DRIVE_CONTROLLER__DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace drive_controller{
  class DriveController : public controller_interface::ControllerInterface{
    public:
      DriveController();

      controller_interface::InterfaceConfiguration command_interface_configuration() const override;
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;

      controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

      controller_interface::CallbackReturn on_init() override;

      controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
      controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    protected:
      std::vector<std::string> joint_names_;
      std::vector<std::string> command_interface_types_;
      std::vector<std::string> state_interface_types_;

      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_sub_;
      rclcpp::Time start_time_;
      float cmd_speed_ = 0;
      float cmd_angle_ = 0;

      float left_wheel_cmd;
      float right_wheel_cmd;

      std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_velocity_command_interface_;

      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_velocity_state_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_position_state_interface_;

      std::unordered_map<
        std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
        command_interface_map_ = {
          {"velocity", &joint_velocity_command_interface_}};

      std::unordered_map<
        std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
        state_interface_map_ = {
          {"velocity", &joint_velocity_state_interface_},
          {"position", &joint_position_state_interface_}
        }; 
  };

}  // namespace drive_controller

#endif  // LOTTI_DRIVE_CONTROLLER__DRIVE_CONTROLLER_HPP_
