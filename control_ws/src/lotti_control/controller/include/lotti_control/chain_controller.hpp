#ifndef LOTTI_CONTROL__CHAIN_CONTROLLER_HPP_
#define LOTTI_CONTROL__CHAIN_CONTROLLER_HPP_

#include <chrono>
#include <memory>
//#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

//#include "control_msgs/action/follow_joint_trajectory.hpp"
//#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/vector3.hpp"


namespace chain_controller
{
  class ChainController : public controller_interface::ControllerInterface
  {
    public:
      ChainController();

      controller_interface::InterfaceConfiguration command_interface_configuration() const override;

      controller_interface::InterfaceConfiguration state_interface_configuration() const override;

      controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

      controller_interface::CallbackReturn on_init() override;

      controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

      controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

      controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    protected:
      std::vector<std::string> joint_names_;
      std::vector<std::string> command_interface_types_;
      std::vector<std::string> state_interface_types_;

      rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr chain_sub_;
      rclcpp::Time start_time_;
      float x_cmd = 0;
      float y_cmd = 0;
      float x_speed = 0;
      float y_speed = 0;     
      float left_chain_cmd = 0;
      float right_chain_cmd = 0;
      float max_speed = 23 * 0.19;
      float wheel_sep = 0.2;
      
      std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_velocity_command_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_torque_command_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_position_state_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_velocity_state_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_torque_state_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_temp_state_interface_;

      std::unordered_map<
        std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
        command_interface_map_ = {
          {"velocity", &joint_velocity_command_interface_},
          {"torque", &joint_torque_command_interface_}};

      std::unordered_map<
        std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
        state_interface_map_ = {
          {"position", &joint_position_state_interface_},
          {"velocity", &joint_velocity_state_interface_},
          {"torque", &joint_torque_state_interface_},
          {"temp", &joint_temp_state_interface_}};
  };

}  // namespace chain_controller

#endif  // LOTTI_CONTROL__CHAIN_CONTROLLER_HPP_
