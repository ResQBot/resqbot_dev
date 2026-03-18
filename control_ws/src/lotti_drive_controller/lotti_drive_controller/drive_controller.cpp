#include "lotti_drive_controller/drive_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <geometry_msgs/msg/twist.hpp>


using config_type = controller_interface::interface_configuration_type;

namespace drive_controller{
  DriveController::DriveController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn DriveController::on_init(){
    // should have error handling
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration DriveController::command_interface_configuration()
    const{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    conf.names.reserve(joint_names_.size() * command_interface_types_.size());
    for (const auto &joint_name : joint_names_){
      for (const auto &interface_type : command_interface_types_){
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }

    return conf;
  }

  controller_interface::InterfaceConfiguration DriveController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    conf.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto &joint_name : joint_names_){
      for (const auto &interface_type : state_interface_types_){
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }

    return conf;
  }

  controller_interface::CallbackReturn DriveController::on_configure(const rclcpp_lifecycle::State &){
    auto drive_callback = 
      [this](geometry_msgs::msg::Twist drive_msg) -> void {
        cmd_speed_ = drive_msg.linear.x;
        cmd_angle_ = drive_msg.angular.z;
    };

    // Subscribers
    drive_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd/drive", 1, drive_callback);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn DriveController::on_activate(const rclcpp_lifecycle::State &){
    // clear out vectors in case of restart
    joint_velocity_command_interface_.clear();

    // assign command interfaces
    for (auto & interface : command_interfaces_){
      command_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

   /*  // assign state interfaces
    for (auto & interface : state_interfaces_){
      state_interface_map_[interface.get_interface_name()]->push_back(interface);
    } */

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type DriveController::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/){
    
    if (cmd_speed_ < 0.0){
      left_wheel_cmd = std::max(-1.0f, std::min((cmd_speed_ + cmd_angle_), 1.0f));
      right_wheel_cmd = - std::max(-1.0f, std::min((cmd_speed_ - cmd_angle_), 1.0f));
    }  
    else{
      left_wheel_cmd = std::max(-1.0f, std::min((cmd_speed_ - cmd_angle_), 1.0f));
      right_wheel_cmd = - std::max(-1.0f, std::min((cmd_speed_ + cmd_angle_), 1.0f));
    }

    joint_velocity_command_interface_[0].get().set_value(left_wheel_cmd);
    joint_velocity_command_interface_[1].get().set_value(right_wheel_cmd);


    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn DriveController::on_deactivate(const rclcpp_lifecycle::State &){
    release_interfaces();

    return CallbackReturn::SUCCESS;
  }

}  // namespace drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive_controller::DriveController, controller_interface::ControllerInterface)
