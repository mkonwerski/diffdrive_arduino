#include "diffdrive_arduino/diffdrive_arduino.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}

return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");
  RCLCPP_INFO(logger_, "------------------------------------configure");


  time_ = std::chrono::system_clock::now();

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
   RCLCPP_INFO(logger_, "cfg_.left_wheel_name: " + cfg_.left_wheel_name);

   RCLCPP_INFO(logger_, "------------------------------------1");
  
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
   RCLCPP_INFO(logger_, "cfg_.right_wheel_name: " + cfg_.right_wheel_name);

   RCLCPP_INFO(logger_, "------------------------------------2");
  
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  //  RCLCPP_INFO(logger_, "cfg_.loop_rate: " + cfg_.loop_rate);
  
  RCLCPP_INFO(logger_, "------------------------------------3");

  
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
   RCLCPP_INFO(logger_, "cfg_.enc_counts_per_rev: " + cfg_.enc_counts_per_rev);

RCLCPP_INFO(logger_, "------------------------------------7");

  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  //  RCLCPP_INFO(logger_, "l_wheel_.setup: "+ l_wheel_.setup);
  
  RCLCPP_INFO(logger_, "------------------------------------8");

  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  //  RCLCPP_INFO(logger_, "r_wheel_.setup: "+ r_wheel_.setup);
  

RCLCPP_INFO(logger_, "------------------------------------9");

  // Set up the Arduino
   arduino_.setup(); 

   RCLCPP_INFO(logger_, "------------------------------------10");

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;


RCLCPP_INFO(logger_, "!!!------------------------------------");

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  RCLCPP_INFO(logger_, "------------------------------------export_state_interfaces");
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
  RCLCPP_INFO(logger_, "------------------------------------export_command_interfaces");
  // We need to set up a velocity command interface for each wheel
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}


return_type DiffDriveArduino::start()
{
  RCLCPP_INFO(logger_, "------------------------------------start");
  RCLCPP_INFO(logger_, "Starting Controller...");

  arduino_.sendEmptyMsg();
  arduino_.setPidValues(30, 20, 0, 100);

  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type DiffDriveArduino::stop()
{
  RCLCPP_INFO(logger_, "------------------------------------stop");
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::read()
{
  RCLCPP_INFO(logger_, "------------------------------------read");
  // TODO fix chrono duration
  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }


  arduino_.readEncoderValues(0,1);

  double pos_prev = l_wheel_.pos;
  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::write()
{
  RCLCPP_INFO(logger_, "------------------------------------write");  

  arduino_.setMotorValues(l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate, r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate);

  return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)