#include "mini_delta_hardware/mini_delta_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace mini_delta_hardware {

    hardware_interface::CallbackReturn MiniDeltaHardwareInterface::on_init(
        const hardware_interface::HardwareInfo & info) 

    {
    if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
    info_ = info;
    servo1_id_ =10;
    servo2_id_ =11;
    servo3_id_ =12;
    serial_device_ = "/dev/ttyACM0"; // default device path
    servo_driver_= std::make_shared<ArduinoServoDriver>(serial_device_);

    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_states_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(),0.0);
    hw_velocity_commands_.resize(info_.joints.size(), 0.0);


    return hardware_interface::CallbackReturn::SUCCESS;

    }    

    hardware_interface::CallbackReturn MiniDeltaHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & previous_state) 
    {
        (void)previous_state; // ignore unused parameter warning
        if (servo_driver_->init() != 0) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS; // else success
    }

    hardware_interface::CallbackReturn MiniDeltaHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & previous_state) 
    {
        (void)previous_state; // ignore unused parameter warning
        servo_driver_->activate();
    return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MiniDeltaHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & previous_state) 
    {
        (void)previous_state; // ignore unused parameter warning
        servo_driver_->deactivate();
    return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MiniDeltaHardwareInterface::export_state_interfaces()
   {
       std::vector<hardware_interface::StateInterface> state_interfaces;
       for (uint i = 0; i < info_.joints.size(); i++)
      {
         state_interfaces.emplace_back(hardware_interface::StateInterface(
         info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));

         state_interfaces.emplace_back(hardware_interface::StateInterface(
         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
      }

    return state_interfaces;
  }

    std::vector<hardware_interface::CommandInterface> MiniDeltaHardwareInterface::export_command_interfaces()
   {
      std::vector<hardware_interface::CommandInterface> command_interfaces;
      for (uint i = 0; i < info_.joints.size(); i++)
     {
         command_interfaces.emplace_back(hardware_interface::CommandInterface(
         info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));


         //adding velocity interface to control stepper freq at arduino this is for stepper only if using servo this line is not important
         command_interfaces.emplace_back( hardware_interface::CommandInterface(
         info_.joints[i].name,hardware_interface::HW_IF_VELOCITY,&hw_velocity_commands_[i]));
     }

    return command_interfaces;
   }
    
          //most important 
    hardware_interface::return_type MiniDeltaHardwareInterface::read(
        const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        (void)time;
        (void)period;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            hw_states_[i]=hw_positions_[i];
            hw_velocities_[i] = hw_velocity_commands_[i];
        } 
        
        // Read positions from servos (not implemented)
    return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MiniDeltaHardwareInterface::write(
        const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        (void)time;
        (void)period;
        double rad_joint1 = hw_positions_[0];
        double rad_joint2 = hw_positions_[1];
        double rad_joint3 = hw_positions_[2];
        //these thre velocity joints are just for steppers
        double vel_joint1 = hw_velocity_commands_[0];
        double vel_joint2 = hw_velocity_commands_[1];
        double vel_joint3 = hw_velocity_commands_[2];

        // In write() temporarily
        std::cout << "VEL COMMANDS: "
                << hw_velocity_commands_[0] << " "
                << hw_velocity_commands_[1] << " "
                << hw_velocity_commands_[2] << std::endl;

        // servo_driver_->setTargetPositions(rad_joint1, rad_joint2, rad_joint3);
        servo_driver_->setTargetPositions(rad_joint1, rad_joint2, rad_joint3,vel_joint1,vel_joint2,vel_joint3);
        

    return hardware_interface::return_type::OK;
    }

} // namespace mini_delta_hardware

// to make our class a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mini_delta_hardware::MiniDeltaHardwareInterface,
  hardware_interface::SystemInterface)

