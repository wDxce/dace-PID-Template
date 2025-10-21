#include "Dace-Template/drive/drive.hpp"
#include "Dace-Template/PID/pid.hpp"
#include <cmath>

constexpr double PI = 3.141592653589793;

template <typename T>
static inline T clamp(T value, T low, T high) {
    return (value < low) ? low : (value > high) ? high : value;
}

namespace dace {

    Drive::Drive(const std::vector<int>& leftPorts,
                 const std::vector<int>& rightPorts,
                 int imuPort,
                 double wheelDiameter,
                 int wheelRPM)
        : imu(imuPort), wheelDiam(wheelDiameter), rpm(wheelRPM) {

        // Normalize ports: use absolute port for construction, set reversed via API.
        auto addMotor = [](std::vector<pros::Motor>& bank, int p) {
            const int port = std::abs(p);
            if (port < 1 || port > 21) {
            }
            bank.emplace_back(port);
            bank.back().set_reversed(p < 0);
            bank.back().set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        };

        for (int p : leftPorts)  addMotor(leftMotors,  p);
        for (int p : rightPorts) addMotor(rightMotors, p);

        // IMPORTANT: Do not reset/calibrate IMU here (global constructor time).
        // Call begin() from initialize() instead.
    }

    // Call this once from initialize()
    void Drive::begin() {
        imu.reset();
        while (imu.is_calibrating()) pros::delay(10);
    }

    void Drive::setBrakeMode(pros::motor_brake_mode_e mode) {
        for (auto& motor : leftMotors)  motor.set_brake_mode(mode);
        for (auto& motor : rightMotors) motor.set_brake_mode(mode);
    }

    pros::Imu& Drive::getIMU() {
        return imu;
    }

    // setTank expects commands in -127..127 and uses pros::Motor::move()
    void Drive::setTank(double leftCmd, double rightCmd) {
        const int l = static_cast<int>(clamp(leftCmd,  -127.0, 127.0));
        const int r = static_cast<int>(clamp(rightCmd, -127.0, 127.0));
        for (auto& m : leftMotors)  m.move(l);
        for (auto& m : rightMotors) m.move(r);
    }

    void Drive::drive(double distance_in, int drive_speed) {
        if (isRunning) wait();

        param1 = distance_in;   // inches (positive fwd, negative back)
        param2 = drive_speed;   // speed cap in -127..127
        currentMotion = MotionType::Drive;
        isRunning = true;
        motionTask = new pros::Task(taskEntry, this);
    }

    void Drive::turn_to_heading(double theta, int turn_speed) {
        if (isRunning) wait();

        param1 = theta;         // target heading (0..360)
        param2 = turn_speed;    // speed cap in -127..127
        currentMotion = MotionType::Turn;
        isRunning = true;
        motionTask = new pros::Task(taskEntry, this);
    }

    void Drive::swing(swing::Direction dir, double theta, int speed) {
        if (isRunning) wait();

        swingDir = dir;
        param1 = theta;         // target heading
        param2 = speed;         // speed cap in -127..127
        currentMotion = MotionType::Swing;
        isRunning = true;
        motionTask = new pros::Task(taskEntry, this);
    }

    void Drive::curve(double rightSpeed, double leftSpeed, double theta) {
        if (isRunning) wait();

        // rightSpeed / leftSpeed are base speeds in -127..127
        param1 = rightSpeed;
        param2 = leftSpeed;
        intParam = static_cast<int>(theta); // target heading
        currentMotion = MotionType::Curve;
        isRunning = true;
        motionTask = new pros::Task(taskEntry, this);
    }

    void Drive::wait() {
        if (motionTask) {
            while (isRunning) pros::delay(10);
            delete motionTask;
            motionTask = nullptr;
        }
    }

    // Static entry point for the task
    void Drive::taskEntry(void* ptr) {
        static_cast<Drive*>(ptr)->motionLoop();
    }

    // Helpers
    static inline double avgPosition(const std::vector<pros::Motor>& bank) {
        if (bank.empty()) return 0.0;
        double sum = 0.0;
        for (const auto& m : bank) sum += m.get_position(); // degrees
        return sum / static_cast<double>(bank.size());
    }

    void Drive::motionLoop() {
        switch (currentMotion) {
            case MotionType::Drive: {
                // Convert inches to encoder degrees
                // rev per inch = 1 / circumference; deg per inch = 360 / circumference
                const double CIRC = wheelDiam * PI;                 // inches per wheel rev
                const double DEG_PER_INCH = 360.0 / CIRC;           // deg per inch

                const double targetDeg = param1 * DEG_PER_INCH;     // may be negative
                // zero encoders
                for (auto& m : leftMotors)  m.tare_position();
                for (auto& m : rightMotors) m.tare_position();

                PID drivePID(drivePIDVals.kP, drivePIDVals.kI, drivePIDVals.kD);
                drivePID.setTarget(targetDeg);

                while (!drivePID.isSettled()) {
                    const double leftDeg  = avgPosition(leftMotors);
                    const double rightDeg = avgPosition(rightMotors);
                    const double currentDeg = (leftDeg + rightDeg) * 0.5;

                    double output = drivePID.calculate(currentDeg);   // expected -127..127 range after tuning
                    output = clamp(output,
                                   -static_cast<double>(std::abs(param2)),
                                    static_cast<double>(std::abs(param2)));

                    setTank(output, output);
                    pros::delay(10);
                }
                break;
            }

            case MotionType::Turn: {
                PID turnPID(turnPIDVals.kP, turnPIDVals.kI, turnPIDVals.kD);
                turnPID.setHeadingMode(true);
                turnPID.setTarget(param1); 

                while (!turnPID.isSettled()) {
                    const double heading = imu.get_heading(); 
                    double power = turnPID.calculate(heading);

                    power = clamp(power,
                                  -static_cast<double>(param2),
                                   static_cast<double>(param2));

                    setTank(-power, power);
                    pros::delay(10);
                }
                break;
            }

            case MotionType::Swing: {
                PID swingPID(swingPIDVals.kP, swingPIDVals.kI, swingPIDVals.kD);
                swingPID.setHeadingMode(true);
                swingPID.setTarget(param1);

                while (!swingPID.isSettled()) {
                    const double heading = imu.get_heading();
                    double power = swingPID.calculate(heading);

                    power = clamp(power,
                                  -static_cast<double>(param2),
                                   static_cast<double>(param2));

                    if (swingDir == swing::Left) {
                        setTank(0, power);
                    } else {
                        setTank(-power, 0);
                    }
                    pros::delay(10);
                }
                break;
            }

            case MotionType::Curve: {
                PID curvePID(curvePIDVals.kP, curvePIDVals.kI, curvePIDVals.kD);
                curvePID.setHeadingMode(true);
                curvePID.setTarget(intParam); // target heading

                while (!curvePID.isSettled()) {
                    const double heading = imu.get_heading();
                    const double correction = curvePID.calculate(heading);

                    // Base speeds are param2 (left) and param1 (right)
                    double leftCmd  = static_cast<double>(param2) - correction;
                    double rightCmd = static_cast<double>(param1) + correction;

                    leftCmd  = clamp(leftCmd,  -127.0, 127.0);
                    rightCmd = clamp(rightCmd, -127.0, 127.0);

                    setTank(leftCmd, rightCmd);
                    pros::delay(10);
                }
                break;
            }

            default:
                break;
        }

        setTank(0, 0);
        isRunning = false;
        currentMotion = MotionType::None;
    }

} // namespace dace
