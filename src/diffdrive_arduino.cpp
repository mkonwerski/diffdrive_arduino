#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

//----------- my code --------------------------------------------
#include "diffdrive_arduino/HBridge.h"
#include "rclcpp/rclcpp.hpp"

  int pi_;
  int pin_;
  HBridge right_;
  HBridge left_;
//----------------------------------------------------------------

DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}

return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");
  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

//----------- my code --------------------------------------------
  
  pi_ = pigpio_start(NULL, NULL);     

  if (pi_ >= 0)
    {
      right_.setPin(pi_, 17, 18);
      left_.setPin(pi_, 27, 22);
    }
    else
    {
      RCLCPP_INFO(logger_, "cannot connect pigpiod");
      rclcpp::shutdown();
      exit(1);
    }   
//-------------------------------------------------------

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));
  return command_interfaces;
}

return_type DiffDriveArduino::start()
{
  return return_type::OK;
}

return_type DiffDriveArduino::stop()
{
  //----------- my code -----------------------------------
  RCLCPP_INFO(logger_, "stop: -----STOP-----");
  
  right_.drive(0);
  left_.drive(0);
  return return_type::OK; 
  //-------------------------------------------------------
}

hardware_interface::return_type DiffDriveArduino::read()
{
  RCLCPP_INFO(logger_, "read: ---------------------------------------");
  return return_type::OK; 
}

hardware_interface::return_type DiffDriveArduino::write()
{
  //----------- my code -----------------------------------
  RCLCPP_INFO(logger_, "write: -----GO!-----");

  right_.drive(255);
  left_.drive(255);

  return return_type::OK; 
  //-------------------------------------------------------
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)
