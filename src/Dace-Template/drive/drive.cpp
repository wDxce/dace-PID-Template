#include "Dace-Template/drive/drive.hpp"
#include "Dace-Template/PID/pid.hpp"
#include <cmath>

constexpr double PI = 3.141592653589793;

template <typename T>
static inline T clamp(T value, T low, T high) {
    return (value < low) ? low : (value > high) ? high : value;
}

// ---- Local helpers (TU-private) ----
static inline double wrap180(double e) {
    while (e > 180.0) e -= 360.0;
    while (e < -180.0) e += 360.0;
    return e;
}
static inline bool timedOut(uint32_t t0, uint32_t limitMs) {
    return (pros::millis() - t0) > limitMs;
}
static inline double gearsetToBaseMotorRPM(pros::v5::MotorGears g) {
    using pros::v5::MotorGears;
    switch (g) {
        case MotorGears::red:   return 100.0; // 36:1
        case MotorGears::green: return 200.0; // 18:1
        case MotorGears::blue:  return 600.0; // 6:1
        default:                return 200.0;
    }
}

namespace dace {

    // NOTE: 'cartridge' parameter is accepted to match the header,
    // but not stored (header currently has no member for it).
    Drive::Drive(const std::vector<int>& leftPorts,
                 const std::vector<int>& rightPorts,
                 int imuPort,
                 double wheelDiameter,
                 int wheelRPM,
                 pros::v5::MotorGears /*cartridge*/)
        : imu(imuPort),
          wheelDiam(wheelDiameter),
          rpm(wheelRPM),
          driveGear(1.0) {

        // Normalize ports: use absolute number and set reverse flag from sign.
        auto addMotor = [](std::vector<pros::Motor>& bank, int p) {
            const int port = std::abs(p);
            if (port < 1 || port > 21) return; // defensive
            bank.emplace_back(port);
            bank.back().set_reversed(p < 0);
            bank.back().set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        };

        for (int p : leftPorts)  addMotor(leftMotors,  p);
        for (int p : rightPorts) addMotor(rightMotors, p);

        // Do not calibrate IMU here (global construction time) — call begin().
    }

    // Call this once from initialize()
    void Drive::begin() {
        imu.reset();
        while (imu.is_calibrating()) pros::delay(10);

        // Auto-detect external drive ratio using detected cartridge gearing and ctor wheel RPM.
        try {
            if (!leftMotors.empty()) {
                const auto g = leftMotors.front().get_gearing();            // pros::v5::MotorGears
                const double baseMotorRPM = gearsetToBaseMotorRPM(g);       // 100/200/600
                if (rpm > 0 && baseMotorRPM > 0) {
                    // rpm argument is interpreted as *wheel* RPM.
                    driveGear = baseMotorRPM / static_cast<double>(rpm);    // motorTurns per wheelTurn
                    if (driveGear < 0.1 || driveGear > 20.0) driveGear = 1.0;
                } else {
                    driveGear = 1.0;
                }
            }
        } catch (...) {
            driveGear = 1.0;
        }
    }

    void Drive::abort() {
        cancel = true;

        if (motionTask) {
            while (isRunning) pros::delay(5);
            delete motionTask;
            motionTask = nullptr;
        }

        setTank(0, 0);
        cancel = false;
        currentMotion = MotionType::None;
    }

    void Drive::idleCoast() {
        setTank(0, 0);
        setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    }

    void Drive::idleBrake() {
        setTank(0, 0);
        setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }

    void Drive::setBrakeMode(pros::motor_brake_mode_e mode) {
        for (auto& m : leftMotors)  m.set_brake_mode(mode);
        for (auto& m : rightMotors) m.set_brake_mode(mode);
    }

    pros::Imu& Drive::getIMU() { return imu; }

    // setTank expects commands in -127..127 and uses pros::Motor::move()
    void Drive::setTank(double leftCmd, double rightCmd) {
        const int l = static_cast<int>(clamp(leftCmd,  -127.0, 127.0));
        const int r = static_cast<int>(clamp(rightCmd, -127.0, 127.0));
        for (auto& m : leftMotors)  m.move(l);
        for (auto& m : rightMotors) m.move(r);
    }

    void Drive::drive(double distance_in, int drive_speed) {
        if (isRunning) wait();
        param1 = distance_in;   // inches (+ forward, - reverse)
        param2 = drive_speed;   // cap in -127..127
        currentMotion = MotionType::Drive;
        isRunning = true;
        motionTask = new pros::Task(taskEntry, this);
    }

    void Drive::turn_to_heading(double theta, int turn_speed) {
        if (isRunning) wait();
        param1 = theta;         // target heading 0..360
        param2 = turn_speed;    // cap in -127..127
        currentMotion = MotionType::Turn;
        isRunning = true;
        motionTask = new pros::Task(taskEntry, this);
    }

    void Drive::swing(swing::Direction dir, double theta, int speed) {
        if (isRunning) wait();
        swingDir = dir;
        param1 = theta;         // target heading
        param2 = speed;         // cap
        currentMotion = MotionType::Swing;
        isRunning = true;
        motionTask = new pros::Task(taskEntry, this);
    }

    void Drive::curve(double rightSpeed, double leftSpeed, double theta) {
        if (isRunning) wait();
        param1 = rightSpeed;                // base right
        param2 = leftSpeed;                 // base left
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
    void Drive::taskEntry(void* ptr) { static_cast<Drive*>(ptr)->motionLoop(); }

    // Helpers
    static inline double avgPosition(const std::vector<pros::Motor>& bank) {
        if (bank.empty()) return 0.0;
        double sum = 0.0;
        for (const auto& m : bank) sum += m.get_position(); // degrees
        return sum / static_cast<double>(bank.size());
    }

    void Drive::motionLoop() {
        switch (currentMotion) {

            // ---------------- DRIVE (distance) ----------------
            case MotionType::Drive: {
                // motorDeg per wheelRev = 360 * driveGear (motorTurns per wheelTurn)
                const double CIRC = wheelDiam * PI;                           // inches / wheel rev
                const double MOTOR_DEG_PER_WHEEL_REV = 360.0 * driveGear;     // motor deg / wheel rev
                const double DEG_PER_INCH = MOTOR_DEG_PER_WHEEL_REV / CIRC;   // motor deg / inch

                const double targetDeg = param1 * DEG_PER_INCH;

                // Zero encoders
                for (auto& m : leftMotors)  m.tare_position();
                for (auto& m : rightMotors) m.tare_position();

                PID drivePID(drivePIDVals.kP, drivePIDVals.kI, drivePIDVals.kD);
                drivePID.setTarget(targetDeg);

                // Safety & straightness
                const uint32_t t0 = pros::millis();
                const uint32_t kTimeout = 8000;        // ms
                const double holdHeading = imu.get_heading();
                const double kH = 0.6;                 // small heading hold

                // Robust settle: distance tolerance + low velocity for N cycles
                const double inchTol = 0.5;                           // stop within ~0.5"
                const double degTol  = inchTol * DEG_PER_INCH;        // motor deg
                const double velTol  = 5.0;                           // motor deg per loop (~10ms)
                const int stableCyclesNeeded = 8;                     // ~80ms
                int stableCycles = 0;
                double lastDeg = 0.0;

                while (true) {
                    if (cancel) break;

                    const double leftDeg  = avgPosition(leftMotors);
                    const double rightDeg = avgPosition(rightMotors);
                    const double currentDeg = 0.5 * (leftDeg + rightDeg);

                    const double err = targetDeg - currentDeg;
                    const double vel = currentDeg - lastDeg; // deg per 10ms
                    lastDeg = currentDeg;

                    const bool inErr  = std::fabs(err) <= degTol;
                    const bool inVel  = std::fabs(vel) <= velTol;
                    if (inErr && inVel) {
                        if (++stableCycles >= stableCyclesNeeded) break;
                    } else {
                        stableCycles = 0;
                    }
                    if (timedOut(t0, kTimeout)) break;

                    double baseOut = drivePID.calculate(currentDeg); // -127..127 after tuning
                    const double cap = static_cast<double>(std::abs(param2));
                    baseOut = clamp(baseOut, -cap, cap);

                    // Straightness correction
                    const double eH = wrap180(holdHeading - imu.get_heading());
                    double leftCmd  = clamp(baseOut - kH * eH, -cap, cap);
                    double rightCmd = clamp(baseOut + kH * eH, -cap, cap);

                    setTank(leftCmd, rightCmd);
                    pros::delay(10);
                }
                break;
            }

            // ---------------- TURN (heading) ----------------
            case MotionType::Turn: {
                PID turnPID(turnPIDVals.kP, turnPIDVals.kI, turnPIDVals.kD);
                turnPID.setHeadingMode(true);
                turnPID.setTarget(param1); // 0..360

                const uint32_t t0 = pros::millis();
                const uint32_t kTimeout = 6000;
                const double cap = std::max(30.0, static_cast<double>(std::abs(param2))); // ensure budge power

                // Manual settle (don’t rely on pid.isSettled())
                const double angTol = 0.6;  // deg
                const double velTol = 0.8;  // deg per loop
                int stableCycles = 0;
                double lastHdg = imu.get_heading();

                while (true) {
                    if (cancel) break;
                    if (timedOut(t0, kTimeout)) break;

                    const double heading = imu.get_heading();
                    const double err = wrap180(param1 - heading);
                    const double vel = wrap180(heading - lastHdg);
                    lastHdg = heading;

                    if (std::fabs(err) <= angTol && std::fabs(vel) <= velTol) {
                        if (++stableCycles >= 8) break;
                    } else {
                        stableCycles = 0;
                    }

                    double power = turnPID.calculate(heading);
                    power = clamp(power, -cap, cap);

                    setTank(-power, power);
                    pros::delay(10);
                }
                break;
            }

            // ---------------- SWING (one side anchored) ----------------
            case MotionType::Swing: {
                PID swingPID(swingPIDVals.kP, swingPIDVals.kI, swingPIDVals.kD);
                swingPID.setHeadingMode(true);
                swingPID.setTarget(param1);
            
                const uint32_t t0 = pros::millis();
                const uint32_t kTimeout = 6000;
            
                // Cap and minimum power to overcome static friction
                const double cap = std::max(30.0, static_cast<double>(std::abs(param2)));
                const double minPower = 25.0; // tweak 20–35 depending on your build
            
                // Manual settle (angle error + low angular velocity for a few ticks)
                const double angTol = 0.6;  // deg
                const double velTol = 0.8;  // deg per loop (~10ms)
                int stableCycles = 0;
                double lastHdg = imu.get_heading();
            
                while (true) {
                    if (cancel) break;
                    if (timedOut(t0, kTimeout)) break;
                
                    const double heading = imu.get_heading();
                    const double err = wrap180(param1 - heading);
                    const double vel = wrap180(heading - lastHdg);
                    lastHdg = heading;
                
                    if (std::fabs(err) <= angTol && std::fabs(vel) <= velTol) {
                        if (++stableCycles >= 8) break;      // settled
                    } else {
                        stableCycles = 0;
                    }
                
                    // PID output
                    double power = swingPID.calculate(heading);
                    power = clamp(power, -cap, cap);
                
                    // Enforce a minimum output if not yet within tolerance
                    if (std::fabs(err) > angTol && std::fabs(power) < minPower) {
                        power = (power >= 0.0 ? minPower : -minPower);
                    }
                
                    if (swingDir == swing::Left) {
                        // Pivot on LEFT: left side 0, drive right side with 'power'
                        setTank(0, power);
                    } else {
                        // Pivot on RIGHT: right side 0, drive left side with '-power'
                        setTank(-power, 0);
                    }
                
                    pros::delay(10);
                }
                break;
            }

            // ---------------- CURVE (drive with heading correction) ----------------
            case MotionType::Curve: {
                PID curvePID(curvePIDVals.kP, curvePIDVals.kI, curvePIDVals.kD);
                curvePID.setHeadingMode(true);
                curvePID.setTarget(intParam); // target heading
            
                const uint32_t t0 = pros::millis();
                const uint32_t kTimeout = 7000;
            
                // Base speeds are param2 (left) and param1 (right) in -127..127
                double baseLeft  = static_cast<double>(param2);
                double baseRight = static_cast<double>(param1);
            
                // Ensure the chassis actually rolls while not settled
                const double minBase = 25.0;  // tweak 20–35 if needed
                const double minCorr = 18.0;  // minimum correction when error exists
            
                // Manual settle
                const double angTol = 0.8;    // deg
                const double velTol = 1.0;    // deg per loop (~10ms)
                int stableCycles = 0;
                double lastHdg = imu.get_heading();
            
                while (true) {
                    if (cancel) break;
                    if (timedOut(t0, kTimeout)) break;
                
                    const double heading = imu.get_heading();
                    const double err = wrap180(static_cast<double>(intParam) - heading);
                    const double vel = wrap180(heading - lastHdg);
                    lastHdg = heading;
                
                    // Settle only when both small error and slow heading change for a few ticks
                    if (std::fabs(err) <= angTol && std::fabs(vel) <= velTol) {
                        if (++stableCycles >= 8) break;
                    } else {
                        stableCycles = 0;
                    }
                
                    // Compute PID correction from heading
                    double correction = curvePID.calculate(heading);
                
                    // If we still have noticeable error but correction is too tiny to move things, bump it
                    if (std::fabs(err) > angTol && std::fabs(correction) < minCorr) {
                        correction = (correction >= 0.0 ? minCorr : -minCorr);
                    }
                
                    // Ensure some forward (or backward) motion when not settled
                    if (std::fabs(err) > angTol) {
                        if (std::fabs(baseLeft)  < minBase) baseLeft  = (baseLeft  >= 0.0 ? minBase : -minBase);
                        if (std::fabs(baseRight) < minBase) baseRight = (baseRight >= 0.0 ? minBase : -minBase);
                    }
                
                    // Apply correction: left gets -correction, right gets +correction
                    double leftCmd  = clamp(baseLeft  - correction, -127.0, 127.0);
                    double rightCmd = clamp(baseRight + correction, -127.0, 127.0);
                
                    setTank(leftCmd, rightCmd);
                    pros::delay(10);
                }
                break;
            }

            default: break;
        }

        setTank(0, 0);
        isRunning = false;
        currentMotion = MotionType::None;
    }

} // namespace dace
