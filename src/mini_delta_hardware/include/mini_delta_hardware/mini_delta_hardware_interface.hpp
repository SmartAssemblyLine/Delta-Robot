#ifndef MINI_DELTA_HARDWARE_INTERFACE_HPP
#define MINI_DELTA_HARDWARE_INTERFACE_HPP



#include "hardware_interface/system_interface.hpp"
#include "mini_delta_hardware/ArduinoServoDriver.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <vector>

namespace mini_delta_hardware {
class MiniDeltaHardwareInterface : public hardware_interface::SystemInterface // inheritance 
{
public:
  //lifecycle callbacks overrides
    hardware_interface::CallbackReturn 
         on_configure(const rclcpp_lifecycle::State & previous_state) override;
    
    hardware_interface::CallbackReturn 
         on_activate(const rclcpp_lifecycle::State & previous_state) override;
    
    hardware_interface::CallbackReturn 
         on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    
    //system interface overrides
    hardware_interface::CallbackReturn
         on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::return_type
         read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type
         write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    std::shared_ptr<ArduinoServoDriver> servo_driver_;  // pointer to servo driver
    int servo1_id_; // servo identifiers
    int servo2_id_;
    int servo3_id_;
    std::string serial_device_;  // serial device path

    std::vector<double> hw_positions_;
    std::vector<double> hw_velocity_commands_;
    std::vector<double> hw_states_;
    std::vector<double> hw_velocities_;
    std::shared_ptr<rclcpp::Node> mission_node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pick_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pump_sub_;
}; // class MiniDeltaHardwareInterface

} // namespace mini_delta_hardware


#endif