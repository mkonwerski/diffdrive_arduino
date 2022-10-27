#include "diffdrive_arduino/diffdrive_arduino.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{
hardware_interface::CallbackReturn DiffDriveArduino::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

 RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------*");
 RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "step: on_init");

  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------1");
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduino"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------12");
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduino"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
     
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------13");
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduino"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------*");
   RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "step: export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------!");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------*");
   RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "step: export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------!");
  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------*");
   RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "step: on_activate");
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Activating ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Successfully activated!");

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------*");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "step: on_deactivate");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduino::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------*");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "step: read");

  double radius = 0.02;  // radius of the wheels
  double dist_w = 0.1;   // distance between the wheels
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[1] + period.seconds() * hw_commands_[i];
    hw_velocities_[i] = hw_commands_[i];

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
      rclcpp::get_logger("DiffDriveArduino"),
      "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());
    // END: This part here is for exemplary purposes - Please do not copy to your production code
  }

  // Update the free-flyer, i.e. the base notation using the classical
  // wheel differentiable kinematics
  double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
  double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
  base_x_ += base_dx * period.seconds();
  base_y_ += base_dy * period.seconds();
  base_theta_ += base_dtheta * period.seconds();

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveArduino"), "Joints successfully read! (%.5f,%.5f,%.5f)",
    base_x_, base_y_, base_theta_);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_arduino::DiffDriveArduino::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------*");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "step: write");

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("DiffDriveArduino"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduino"), "-----------------------------!");
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduino, hardware_interface::SystemInterface)