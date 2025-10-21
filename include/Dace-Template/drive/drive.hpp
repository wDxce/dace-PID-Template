#pragma once
#include <vector>
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

/*
Drive Header File
*/

namespace swing {
    enum Direction { Left, Right };
}

namespace dace {

    class Drive {
    public:
        Drive(const std::vector<int>& leftPorts,
              const std::vector<int>& rightPorts,
              int imuPort,
              double wheelDiameter,
              int wheelRPM);

        // Call once in initialize() to calibrate IMU safely (not in global ctor)
        void begin();

        pros::Imu& getIMU();

        // Applies to all drive motors
        void setBrakeMode(pros::motor_brake_mode_e mode);

        // Tank command input expected in -127..127 range (mapped via Motor::move)
        void setTank(double leftVoltage, double rightVoltage);

        // Async motion commands (use wait() to block until complete)
        void drive(double distance_in, int drive_speed);
        void turn_to_heading(double theta, int turn_speed);
        void swing(swing::Direction dir, double theta, int speed);
        void curve(double rightSpeed, double leftSpeed, double theta);

        // Block until current motion (if any) completes
        void wait();

    private:
        std::vector<pros::Motor> leftMotors;
        std::vector<pros::Motor> rightMotors;
        pros::Imu imu;

        double wheelDiam;
        int rpm;

        pros::Task* motionTask = nullptr;
        bool isRunning = false;

        enum class MotionType { None, Drive, Turn, Swing, Curve };
        MotionType currentMotion = MotionType::None;

        // Generic params passed to the motion task
        double param1 = 0.0;
        double param2 = 0.0;
        int    intParam = 0;
        swing::Direction swingDir = swing::Left;

        // Internal task loop & entry
        void motionLoop();
        static void taskEntry(void* ptr);
    };

} // namespace dace
