#include "sanad_firmware/sanad_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace sanad_firmware
{
SanadInterface::SanadInterface()
{
}


SanadInterface::~SanadInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("SanadInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn SanadInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("SanadInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  // resize() allocates AND zero-initialises; reserve() only reserves capacity
  // and leaves size=0, so at(0)/at(1) would throw std::out_of_range.
  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(),   0.0);
  velocity_states_.resize(info_.joints.size(),    0.0);

  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> SanadInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> SanadInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn SanadInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SanadInterface"), "Starting robot hardware ...");

  velocity_commands_ = { 0.0, 0.0 };
  position_states_   = { 0.0, 0.0 };
  velocity_states_   = { 0.0, 0.0 };

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("SanadInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("SanadInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn SanadInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SanadInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("SanadInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SanadInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type SanadInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  if (!arduino_.IsDataAvailable())
  {
    return hardware_interface::return_type::OK;
  }

  try
  {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();

    std::string message;
    arduino_.ReadLine(message);

    // Arduino sends exactly:  "r<sign><abs_value>,l<sign><abs_value>,\n"
    // Serial.print('\n') gives LF only (no CR), so no '\r' to strip.
    // But guard anyway: strip any trailing CR/LF from the line.
    while (!message.empty() && (message.back() == '\n' || message.back() == '\r'))
    {
      message.pop_back();
    }

    std::stringstream ss(message);
    std::string token;

    while (std::getline(ss, token, ','))
    {
      // Need at least: id(1) + sign(1) + one digit(1) = 3 chars minimum
      if (token.size() < 3)
      {
        continue;
      }

      const char wheel_id = token[0];
      const char sign_chr = token[1];

      if (wheel_id != 'r' && wheel_id != 'l')  continue;
      if (sign_chr  != 'p' && sign_chr  != 'n') continue;

      // substr(2) gives the numeric part — always a positive decimal string
      // because the Arduino only passes fabs() values to snprintf.
      // We ONLY apply the sign from sign_chr, never from the number itself.
      double abs_vel = 0.0;
      try
      {
        abs_vel = std::fabs(std::stod(token.substr(2)));
      }
      catch (const std::exception &)
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("SanadInterface"),
                           "Malformed velocity token: \"" << token << "\" — skipped");
        continue;
      }

      // Apply the sign from the protocol sign char only
      const double signed_vel = (sign_chr == 'p') ? abs_vel : -abs_vel;

      if (wheel_id == 'r')
      {
        velocity_states_.at(0)  = signed_vel;
        position_states_.at(0) += signed_vel * dt;
      }
      else
      {
        velocity_states_.at(1)  = signed_vel;
        position_states_.at(1) += signed_vel * dt;
      }
    }

    last_run_ = rclcpp::Clock().now();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SanadInterface"),
                        "Error in read(): " << e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type SanadInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Protocol to Arduino:  "r<sign><abs_value>,l<sign><abs_value>,\n"
  // Example forward/backward: "rp01.50,ln01.50,\n"
  //
  // Format rules that match the Arduino's 5-char value buffer ("XX.XX"):
  //   • always use fabs() — the sign char carries the direction
  //   • prepend '0' when abs value < 10  →  "09.87" not "9.87"
  //   • setprecision(2) gives exactly 2 decimal places
  //   • terminate with '\n' (LF only) to match Serial.print('\n') on Arduino

  const double abs_right = std::fabs(velocity_commands_.at(0));
  const double abs_left  = std::fabs(velocity_commands_.at(1));

  const char right_sign = (velocity_commands_.at(0) >= 0.0) ? 'p' : 'n';
  const char left_sign  = (velocity_commands_.at(1) >= 0.0) ? 'p' : 'n';

  const std::string pad_right = (abs_right < 10.0) ? "0" : "";
  const std::string pad_left  = (abs_left  < 10.0) ? "0" : "";

  std::ostringstream msg;
  msg << std::fixed << std::setprecision(2)
      << "r" << right_sign << pad_right << abs_right
      << ",l" << left_sign  << pad_left  << abs_left
      << ",\n";

  try
  {
    arduino_.Write(msg.str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SanadInterface"),
                        "Error in write(): \"" << msg.str() << "\" → " << e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace sanad_firmware

PLUGINLIB_EXPORT_CLASS(sanad_firmware::SanadInterface, hardware_interface::SystemInterface)