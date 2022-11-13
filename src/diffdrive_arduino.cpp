#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "diffdrive_arduino/HBridge.h"

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

  RCLCPP_INFO(logger_, "configure: ---------------------------------------1");
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  RCLCPP_INFO(logger_, "configure: ---------------------------------------2");
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  RCLCPP_INFO(logger_, "configure: ---------------------------------------3");
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  RCLCPP_INFO(logger_, "configure: ---------------------------------------4");
  cfg_.device = info_.hardware_parameters["device"];
  RCLCPP_INFO(logger_, "configure: ---------------------------------------5");
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  RCLCPP_INFO(logger_, "configure: ---------------------------------------6");
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  RCLCPP_INFO(logger_, "configure: ---------------------------------------7");
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Set up the wheels
  RCLCPP_INFO(logger_, "configure: ---------------------------------------8");
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  RCLCPP_INFO(logger_, "configure: ---------------------------------------9");
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Set up the Arduino
  RCLCPP_INFO(logger_, "configure: ---------------------------------------10");
  // arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  


//----------- my code --------------------------------------------
  int pi_;
  HBridge right_;
  HBridge left_;

  RCLCPP_INFO(logger_, "pigpio_start: ---------------------------------------");
  pi_ = pigpio_start(NULL, NULL);
        
  RCLCPP_INFO(logger_, "pigpio_start: ---------------------------------------1");
  if (pi_ >= 0)
    {
      RCLCPP_INFO(logger_, "pigpio_start: ---------------------------------------2");
      right_.setPin(pi_, 17, 18);
      RCLCPP_INFO(logger_, "pigpio_start: ---------------------------------------3");
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

  RCLCPP_INFO(logger_, "export_state_interfaces: ---------------------------------------");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  RCLCPP_INFO(logger_, "export_state_interfaces: ---------------------------------------1");
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  RCLCPP_INFO(logger_, "export_state_interfaces: ---------------------------------------2");
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  RCLCPP_INFO(logger_, "export_state_interfaces: ---------------------------------------3");
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  RCLCPP_INFO(logger_, "export_state_interfaces: ---------------------------------------4");
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  RCLCPP_INFO(logger_, "export_state_interfaces: ---------------------------------------5");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  RCLCPP_INFO(logger_, "export_command_interfaces: ---------------------------------------");
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  RCLCPP_INFO(logger_, "export_command_interfaces: ---------------------------------------1");
  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  RCLCPP_INFO(logger_, "export_command_interfaces: ---------------------------------------2");
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  RCLCPP_INFO(logger_, "export_command_interfaces: ---------------------------------------3");
  return command_interfaces;
}

return_type DiffDriveArduino::start()
{
  RCLCPP_INFO(logger_, "start: ---------------------------------------");
  return return_type::OK;
}

return_type DiffDriveArduino::stop()
{
  //----------- my code -----------------------------------
  RCLCPP_INFO(logger_, "stop: -----STOP-----");
  HBridge right_;
  HBridge left_;
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
  HBridge right_;
  HBridge left_;
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
