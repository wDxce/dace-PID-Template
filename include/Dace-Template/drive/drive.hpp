#pragma once
#include <vector>
#include <atomic>                 // NEW: for std::atomic
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

/*
Drive Header File
*/

constexpr pros::v5::MotorGears blue  = pros::v5::MotorGears::blue;
constexpr pros::v5::MotorGears green = pros::v5::MotorGears::green;
constexpr pros::v5::MotorGears red   = pros::v5::MotorGears::red;

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
              int wheelRPM,
              pros::v5::MotorGears cartridge);

        // Call once in initialize() to calibrate IMU safely (not in global ctor)
        void begin();

        // Task/motion control
        void abort();        // cancels any ongoing motion task and stops motors
        void idleCoast();    // sets motors to COAST and zeros output
        void idleBrake();    // sets motors to HOLD (or BRAKE) and zeros output
        
        pros::Imu& getIMU();

        // Applies to all drive motors
        void setBrakeMode(pros::motor_brake_mode_e mode);

        // Tank command input expected in -127..127 range (mapped via Motor::move)
        void setTank(double leftCmd, double rightCmd);

        // Async motion commands (use wait() to block until complete)
        void drive(double distance_in, int drive_speed);
        void turn_to_heading(double theta, int turn_speed);
        void swing(swing::Direction dir, double theta, int speed);
        void curve(double rightSpeed, double leftSpeed, double theta);

        // Block until current motion (if any) completes
        void wait();

        void setExternalDriveRatio(double motorTurnsPerWheelTurn) { driveGear = motorTurnsPerWheelTurn; }

    private:
        std::vector<pros::Motor> leftMotors;
        std::vector<pros::Motor> rightMotors;
        pros::Imu imu;

        double wheelDiam;      // inches
        int    rpm;            // wheel RPM passed by user
        double driveGear = 1.0; // motor turns per wheel turn (external ratio)

        pros::Task* motionTask = nullptr;

        // Task state
        std::atomic<bool> isRunning{false}; // was bool; now atomic to avoid races
        std::atomic<bool> cancel{false};    // NEW: used by abort() and loops

        enum class MotionType { None, Drive, Turn, Swing, Curve };
        MotionType currentMotion = MotionType::None;

        // Generic params passed to the motion task
        double           param1 = 0.0;
        double           param2 = 0.0;
        int              intParam = 0;
        swing::Direction swingDir = swing::Left;

        // Internal task loop & entry
        void motionLoop();
        static void taskEntry(void* ptr);
    };

} // namespace dace
