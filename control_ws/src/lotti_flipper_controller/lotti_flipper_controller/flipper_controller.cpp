#include "lotti_flipper_controller/flipper_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <cstddef>
#include <sstream>
#include <iomanip>
#include <vector>
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <std_msgs/msg/int8.hpp>


using config_type = controller_interface::interface_configuration_type;

namespace flipper_controller{
  FlipperController::FlipperController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn FlipperController::on_init(){
    // should have error handling
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration FlipperController::command_interface_configuration()
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

  controller_interface::InterfaceConfiguration FlipperController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    conf.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto &joint_name : joint_names_){
      for (const auto &interface_type : state_interface_types_){
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }

    return conf;
  }

  controller_interface::CallbackReturn FlipperController::on_configure(const rclcpp_lifecycle::State &){
    auto fr_flipper_callback = 
      [this](std_msgs::msg::Int8 fr_flipper_msg) -> void {
        fr_flipper_cmd = fr_flipper_msg.data;
    };

    auto fl_flipper_callback = 
      [this](std_msgs::msg::Int8 fl_flipper_msg) -> void {
        fl_flipper_cmd = fl_flipper_msg.data;
    };

    auto rr_flipper_callback = 
      [this](std_msgs::msg::Int8 rr_flipper_msg) -> void {
        rr_flipper_cmd = rr_flipper_msg.data;
    };

    auto rl_flipper_callback = 
      [this](std_msgs::msg::Int8 rl_flipper_msg) -> void {
        rl_flipper_cmd = rl_flipper_msg.data;
    };

    // Subscribers
    fr_flipper_sub_ =
      get_node()->create_subscription<std_msgs::msg::Int8>(
        "/cmd/flipper_fr", 1, fr_flipper_callback);
        
    fl_flipper_sub_ =
      get_node()->create_subscription<std_msgs::msg::Int8>(
        "/cmd/flipper_fl", 1, fl_flipper_callback);

    rr_flipper_sub_ =
      get_node()->create_subscription<std_msgs::msg::Int8>(
        "/cmd/flipper_rr", 1, rr_flipper_callback);

    rl_flipper_sub_ =
      get_node()->create_subscription<std_msgs::msg::Int8>(
        "/cmd/flipper_rl", 1, rl_flipper_callback);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn FlipperController::on_activate(const rclcpp_lifecycle::State &){
    // clear out vectors in case of restart
    joint_velocity_command_interface_.clear();
    joint_position_state_interface_.clear();

    // assign command interfaces
    for (auto & interface : command_interfaces_){
      command_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    // assign state interfaces
    for (auto & interface : state_interfaces_){
      state_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type FlipperController::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/){

    joint_velocity_command_interface_[1].get().set_value(fr_flipper_cmd);
    joint_velocity_command_interface_[0].get().set_value(fl_flipper_cmd);
    joint_velocity_command_interface_[3].get().set_value(rr_flipper_cmd);
    joint_velocity_command_interface_[2].get().set_value(rl_flipper_cmd);

    //std::cout << std::to_string(fr_flipper_cmd) << "\n";
    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn FlipperController::on_deactivate(const rclcpp_lifecycle::State &){
    release_interfaces();

    return CallbackReturn::SUCCESS;
  }

}  // namespace flipper_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  flipper_controller::FlipperController, controller_interface::ControllerInterface)
