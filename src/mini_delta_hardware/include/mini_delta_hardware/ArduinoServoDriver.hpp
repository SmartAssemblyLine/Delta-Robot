#ifndef ARDUINO_SERVO_DRIVER_HPP
#define ARDUINO_SERVO_DRIVER_HPP

#include <string>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cmath>

class ArduinoServoDriver {
public:
    ArduinoServoDriver(std::string device_name)
        : device_name_(device_name) {}


    int init() {
    std::cout << "Initializing Arduino Servo Driver..." << std::endl;

    try {
        serial_port_.Open(device_name_);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        std::cout << "Serial port opened successfully!" << std::endl;
    } catch (...) {
        std::cout << "Failed to open serial port!" << std::endl;
        return -1;
    }

    // ── Wait for Arduino to send "READY" after boot ──────────────
    std::cout << "Waiting for Arduino READY..." << std::endl;
    std::string line;
    while (true) {
        try {
            serial_port_.ReadLine(line, '\n', 5000);  // 5 sec timeout
            line.erase(line.find_last_not_of(" \r\n") + 1);  // trim
            if (line == "READY") break;
        } catch (...) {
            std::cout << "Timeout waiting for READY!" << std::endl;
            return -1;
        }
    }

    // ── Send START to trigger homing ──────────────────────────────
    // std::cout << "Sending START..." << std::endl;
    // serial_port_.Write("START\n");

    // // ── Wait for HOMED before letting ROS2 control begin ─────────
    // std::cout << "Waiting for homing to complete..." << std::endl;
    // while (true) {
    //     try {
    //         serial_port_.ReadLine(line, '\n', 30000);  // 30 sec timeout
    //         line.erase(line.find_last_not_of(" \r\n") + 1);  // trim
    //         std::cout << "[Arduino] " << line << std::endl;  // shows JOINT_X_HOMED too
    //         if (line == "HOMED") break;
    //     } catch (...) {
    //         std::cout << "Timeout waiting for HOMED!" << std::endl;
    //         return -1;
    //     }
    // }

    // std::cout << "Homing complete! Hardware ready." << std::endl;
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

    ~ArduinoServoDriver() {
        if (serial_port_.IsOpen())
            serial_port_.Close();
    }

private:
    std::string device_name_;
    LibSerial::SerialPort serial_port_;
};

#endif
