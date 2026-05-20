#ifndef LOTTI_FLIPPER3_CONTROLLER__FLIPPER3_CONTROLLER_HPP_
#define LOTTI_FLIPPER3_CONTROLLER__FLIPPER3_CONTROLLER_HPP_

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/int8.hpp"


namespace flipper3_controller{
  class FlipperController : public controller_interface::ControllerInterface{
    public:
      FlipperController();

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

      rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr fr_flipper_sub_;
      rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr fl_flipper_sub_;
      rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr rr_flipper_sub_;
      rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr rl_flipper_sub_;
      rclcpp::Time start_time_;
      int fr_flipper_cmd = 0;
      int fl_flipper_cmd = 0;
      int rr_flipper_cmd = 0;
      int rl_flipper_cmd = 0;

      std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_velocity_command_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_position_state_interface_;

      std::unordered_map<
        std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
        command_interface_map_ = {
          {"velocity", &joint_velocity_command_interface_}};

      std::unordered_map<
        std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
        state_interface_map_ = {
          {"position", &joint_position_state_interface_}};
  };

}  // namespace flipper3_controller

#endif  // LOTTI_FLIPPER_CONTROLLER__FLIPPER_CONTROLLER_HPP_
