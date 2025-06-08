#ifndef FLIPPER_INTERFACE__FLIPPER_INTERFACE_HPP_
#define FLIPPER_INTERFACE__FLIPPER_INTERFACE_HPP_

#include "string"
#include "unordered_map"
#include "vector"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "lotti_control/serial_comms.hpp"

using hardware_interface::return_type;

namespace flipper_interface{
  
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class HARDWARE_INTERFACE_PUBLIC FlipperInterface : public hardware_interface::SystemInterface {
    
    public:
      CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
      hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

      hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;    
      hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

      return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
      return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
    
    protected:
      /// The size of this vector is (standard_interfaces_.size() x nr_joints)
      std::vector<double> joint_velocities_command_;
      std::vector<double> joint_positions_;

      std::string device_ = "";

      float fr_cmd_ = 0;
      float fl_cmd_ = 0;
      float rr_cmd_ = 0;
      float rl_cmd_ = 0;

      float fr_state_ = 0;
      float fl_state_ = 0;
      float rr_state_ = 0;
      float rl_state_ = 0;

      std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
        {"position", {}}, {"velocity", {}}};

      //SerialComms flipper_comms_;
  };
}  // namespace flipper_interface

#endif  // FLIPPER_INTERFACE__FLIPPER_INTERFACE_HPP_
