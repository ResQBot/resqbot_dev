
#ifndef DRIVE_INTERFACE__DRIVE_INTERFACE_HPP_
#define DRIVE_INTERFACE__DRIVE_INTERFACE_HPP_

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

#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

using hardware_interface::return_type;

namespace drive_interface{
  
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class HARDWARE_INTERFACE_PUBLIC DriveInterface : public hardware_interface::SystemInterface {
    
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
      std::vector<double> joint_velocities_;
      std::vector<double> joint_positions_;
      std::vector<double> joint_torques_;
      std::vector<double> joint_temps_;

      std::string device_ = "";
      float max_speed_ = 0.0;
      float max_torque_ = 0.0;
      float gearRatio = 6.33;

      float speed_l = 0.0;
      float speed_r = 0.0;
      int direction_l = 0;
      int direction_r = 0;     
      float torque_l = 0.0;
      float torque_r = 0.0;


      float pos_prev_l = 0.0;
      float pos_prev_r = 0.0;

      std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
        {"position", {}}, {"velocity", {}}, {"torque", {}}, {"temp", {}}};

      // --- Add Unitree SDK members here ---
      std::unique_ptr<SerialPort> serial_;
      MotorCmd cmd_l_, cmd_r_;  // Motor commands for left and right motors
      MotorData data_l_, data_r_; // Motor data feedback for left and right motors
  };
}  // namespace drive_interface

#endif  // DRIVE_INTERFACE__DRIVE_INTERFACE_HPP_
