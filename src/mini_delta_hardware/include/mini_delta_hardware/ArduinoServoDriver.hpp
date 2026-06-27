#ifndef ARDUINO_SERVO_DRIVER_HPP
#define ARDUINO_SERVO_DRIVER_HPP

#include <string>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>

class ArduinoServoDriver {
public:
    ArduinoServoDriver(std::string device_name)
        : device_name_(device_name) {}


    int init() {
    // Open serial port
    std::cout << "Initializing Arduino Servo Driver..." << std::endl;
    RCLCPP_INFO(
        rclcpp::get_logger("ArduinoServoDriver"),
        "Initializing Arduino Servo Driver..."
        );

    try {
        serial_port_.Open(device_name_);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Serial port opened successfully!" << std::endl;
        RCLCPP_INFO(
            rclcpp::get_logger("ArduinoServoDriver"),
            "Serial port opened successfully!"
            );

        
        
    } catch (...) {
        std::cout << "Failed to open serial port!" << std::endl;
        RCLCPP_ERROR(
            rclcpp::get_logger("ArduinoServoDriver"),
            "Failed to open serial port!"
            );
        return -1;
    }



    // ── Wait for Arduino to send "READY" after boot ──────────────
    std::cout << "Waiting for Arduino READY..." << std::endl;
    RCLCPP_INFO(
        rclcpp::get_logger("ArduinoServoDriver"),
        "Waiting for Arduino READY..."
    );
    std::string line;
    bool ready_received = false;
    while (!ready_received) {
        try {
            serial_port_.ReadLine(line, '\n', 5000);  // 5 sec timeout
            line.erase(line.find_last_not_of(" \r\n") + 1);  // trim
            RCLCPP_INFO(
                rclcpp::get_logger("ArduinoServoDriver"),
                "Arduino says: '%s'",
                line.c_str()
            );
            if (line == "READY") 
            {   
                RCLCPP_INFO(
                    rclcpp::get_logger("ArduinoServoDriver"),
                    "Arduino READY received."
                    );
                serial_port_.Write("S\n");  
                serial_port_.DrainWriteBuffer();
                serial_port_.FlushIOBuffers(); 
                ready_received = true;
                break;
            }
        } catch (...) {
            std::cout << "Timeout waiting for READY!" << std::endl;
            RCLCPP_ERROR(
                rclcpp::get_logger("ArduinoServoDriver"),
                "Timeout waiting for READY!"
                );
            return -1;
        }
    }


    RCLCPP_INFO(rclcpp::get_logger("ArduinoServoDriver"), "Waiting for HOMED...");
    while (ready_received) {
        try {
            serial_port_.ReadLine(line, '\n', 30000);
            line.erase(line.find_last_not_of(" \r\n") + 1);
            RCLCPP_INFO(rclcpp::get_logger("ArduinoServoDriver"), "Arduino says: '%s'", line.c_str());

            if (line == "HOMED") {
                RCLCPP_INFO(rclcpp::get_logger("ArduinoServoDriver"), "Arduino HOMED received. Ready to control.");
                break;
            }
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("ArduinoServoDriver"), "Timeout waiting for HOMED!");
            return -1;
        }
    }

   
    return 0;
}

    void activate() {
        std::cout << "Servo driver activated." << std::endl;
    }

    // Deactivate
    void deactivate() {
        std::cout << "Servo driver deactivated." << std::endl;
    }


    void setTargetPositions(double radian1, double radian2, double radian3,
                            double vel1, double vel2, double vel3) {
        if (!serial_port_.IsOpen()) return;

        const int    steps_per_rev = 3200;
        const double two_pi        = 2.0 * M_PI;

        int steps1 = static_cast<int>((radian1 / two_pi) * steps_per_rev*3);
        int steps2 = static_cast<int>((radian2 / two_pi) * steps_per_rev*3);
        int steps3 = static_cast<int>((radian3 / two_pi) * steps_per_rev*3);

        // Just positions — AccelStepper handles velocity/accel
        std::string msg = std::to_string(steps1) + " " +
                        std::to_string(steps2) + " " +
                        std::to_string(steps3) + "\n";

        serial_port_.Write(msg);
    }





    double getPosition(int servo_id) {
      //feedback 
        return 0.0;
    }


    std::string readLine() {
        if (!serial_port_.IsDataAvailable()) return "";
        
        std::string line;
        try {
            serial_port_.ReadLine(line, '\n', 0);  // non-blocking
            line.erase(line.find_last_not_of(" \r\n") + 1);
            RCLCPP_INFO(rclcpp::get_logger("ArduinoServoDriver"), "Arduino says: '%s'", line.c_str());
            return line;
        } catch (...) {
            return "";
        }
    }

    void sendCommand(const std::string & cmd) {
        if (!serial_port_.IsOpen()) return;
        serial_port_.Write(cmd + "\n");
        serial_port_.DrainWriteBuffer();
    }

    ~ArduinoServoDriver() {
        if (serial_port_.IsOpen())
            serial_port_.Close();
    }

private:
    std::string device_name_;
    LibSerial::SerialPort serial_port_;
};

#endif
