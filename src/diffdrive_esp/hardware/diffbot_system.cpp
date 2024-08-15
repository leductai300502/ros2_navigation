#include "diffdrive_esp/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_esp
{
hardware_interface::CallbackReturn DiffDriveEspHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  cfg_.period =  std::stod(info_.hardware_parameters["period"]);

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveEsHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveEspardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveEspHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveEspHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveEspHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> DiffDriveEspHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> DiffDriveEspHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;    
}


hardware_interface::CallbackReturn DiffDriveEspHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveEspHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveEspHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveEspHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveEspHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveEspHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveEspHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveEspHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  first_read_pass_ = first_write_pass_ = true;

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveEspHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveEspHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveEspHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveEspHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type DiffDriveEspHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // ===================== WRITE + READ AT THE SAME TIME + PERIOD ====================================================

  // if (first_write_pass_ || (time - last_write_time_ ).seconds() > cfg_.period)
  // {
  //   first_write_pass_ = false;
  //   last_write_time_ = time;
  //   // hardware comms and operations
  //   double motor_l_rpm = wheel_l_.cmd * 60 / (2*M_PI);
  //   double motor_r_rpm = wheel_r_.cmd * 60 / (2*M_PI);

  //   comms_.set_motor_values2(motor_l_rpm, motor_r_rpm, wheel_l_.vel_rpm, wheel_r_.vel_rpm);
    
  //   wheel_l_.pos = wheel_l_.calc_enc_angle();
  //   wheel_l_.vel = wheel_l_.vel_rpm / 60 * (2*M_PI);

  //   wheel_r_.pos = wheel_r_.calc_enc_angle();
  //   wheel_r_.vel = wheel_r_.vel_rpm / 60 * (2*M_PI);
  // }
  
  //============================ READ STATES VALUE 2 WITH PERIOD =====================================================

  // if (!comms_.connected())
  // {
  //   return hardware_interface::return_type::ERROR;
  // }

  // if (first_read_pass_ || (time - last_read_time_ ).seconds() > cfg_.period)
  // {
  //   first_read_pass_ = false;
  //   last_read_time_ = time;
  //   // hardware comms and operations
  //   comms_.read_states_values2(wheel_l_.vel_rpm, wheel_r_.vel_rpm);
  //   wheel_l_.pos = wheel_l_.calc_enc_angle();
  //   wheel_l_.vel = wheel_l_.vel_rpm / 60 * (2*M_PI);

  //   wheel_r_.pos = wheel_r_.calc_enc_angle();
  //   wheel_r_.vel = wheel_r_.vel_rpm / 60 * (2*M_PI);
  // }

  //===================================== READ STATES VALUE ==========================================================
  
  comms_.read_states_values(wheel_l_.vel_rpm, wheel_l_.enc, wheel_r_.vel_rpm, wheel_r_.enc);

  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = wheel_l_.vel_rpm / 60 * (2*M_PI);

  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = wheel_r_.vel_rpm / 60 * (2*M_PI);

  // ==================================== CALULATE VELOCITY BASE ON ENCODER VALUE =====================================

  // double delta_seconds = period.seconds();

  // double pos_prev = wheel_l_.pos;
  // wheel_l_.pos = wheel_l_.calc_enc_angle();
  // wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  // pos_prev = wheel_r_.pos;
  // wheel_r_.pos = wheel_r_.calc_enc_angle();
  // wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type diffdrive_esp ::DiffDriveEspHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // ========================= WRITE WITH PERIOD =================================================================

  // if (first_write_pass_ || (time - last_write_time_ ).seconds() > cfg_.period)
  // {
  //   first_write_pass_ = false;
  //   last_write_time_ = time;

  //   // hardware comms and operations
  //   double motor_l_rpm = wheel_l_.cmd * 60 / (2*M_PI);
  //   double motor_r_rpm = wheel_r_.cmd * 60 / (2*M_PI);
  //   comms_.set_motor_values(motor_l_rpm, motor_r_rpm);
  // }

  // ==============================================================================================================
  // cmd: rad/s

  double motor_l_rpm = wheel_l_.cmd * 60 / (2*M_PI);
  double motor_r_rpm = wheel_r_.cmd * 60 / (2*M_PI);
  comms_.set_motor_values(motor_l_rpm, motor_r_rpm);

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_esp

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_esp::DiffDriveEspHardware, hardware_interface::SystemInterface)
