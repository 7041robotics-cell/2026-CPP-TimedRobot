#include <iostream>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <rev/SparkFlex.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/BangBangController.h>
#include <frc/XboxController.h>
#include <rev/SparkRelativeEncoder.h>
#include "ctre/phoenix6/StatusSignal.hpp"
#include <frc/MathUtil.h>
#include <algorithm>
#include <frc/motorcontrol/Spark.h>
#include <numbers>
#include <frc/Encoder.h>
#include <cameraserver/CameraServer.h>
#include <cscore.h>
#include <cscore_oo.h>
#include <networktables/NetworkTableInstance.h>
#include <LimelightHelpers.h>
#include <cameraserver/CameraServer.h>
#include <cmath>

using namespace units::literals;

class SwerveModule {
public:
    rev::spark::SparkFlex motor_s; // Steer Motor
    rev::spark::SparkFlex motor_d; // Drive Motor
    ctre::phoenix6::hardware::CANcoder cancoder; // Cancoder
    ctre::phoenix6::StatusSignal<units::angle::turn_t> status_c; // Cancoder status thing (Allows the code to call a refresh of the data from the CAN bus)
    rev::spark::SparkRelativeEncoder encode_s; // Relative Steering Encoder in the Spark Flex
    rev::spark::SparkRelativeEncoder encode_d; // Relative Driving Encoder in the Spark Flex
    frc::PIDController pid_s; // Steering PID
    frc::PIDController pid_d; // Driving PID

    SwerveModule(int corner, double sP, double sI, double sD, double dP, double dI, double dD)
        : motor_s{corner * 10 + 1, rev::spark::SparkLowLevel::MotorType::kBrushless}, // Sets the steer motor to the second number being 2
          motor_d{corner * 10 + 2, rev::spark::SparkLowLevel::MotorType::kBrushless}, // Sets the drive motor to the second number being 2
          cancoder{corner * 10 + 3}, // Sets the CANcoder to the second number being 3
          status_c{cancoder.GetAbsolutePosition()},
          encode_s{motor_s.GetEncoder()}, 
          encode_d{motor_d.GetEncoder()},
          pid_s{sP, sI, sD},
          pid_d{dP, dI, dD}
    { 
        pid_s.EnableContinuousInput(0, 1); // 0 - 1 for CANCoders. This basically tells the steer PID "You're a circle, 0 and 1 are the same"
    }
      // Turns off the steer and drive motor for the given module
      void Stop() {
      motor_s.Set(0);
      motor_d.Set(0);
    }
      // Resets the PIDs for the current module
    void ResetPIDs() {
        pid_s.Reset();
        pid_d.Reset();
    }
    void RefreshCancoder() { // Refreshes the cancoder for the... dude do I have to specifify it's for the current module you get the idea
        status_c.Refresh();

    } // Syncs the Relative Encoder in the turning Spark to the Absolute in the CANCoder (TODO: TEST THIS)
    void SyncEncoderToCancoder(double ratio_s) {
        status_c.Refresh();
        encode_s.SetPosition(status_c.GetValue().value() * ratio_s);
    }
    // This one is probably completelty irrelevant since we're using CANCoders for steering anyways but I'll just leave it
    void WrapEncoder() { 
        if (encode_s.GetPosition() > 360) encode_s.SetPosition(encode_s.GetPosition() - 360);
        if (encode_s.GetPosition() < 0)   encode_s.SetPosition(encode_s.GetPosition() + 360);
    }
    // This is basically the only part that matters
    void Set(frc::SwerveModuleState state, double ratio_s, double ratio_d, double wheel_c) {
        status_c.Refresh(); // Refresh the status for the CANcoders to make sure they're updated
        
        frc::Rotation2d current{units::turn_t(status_c.GetValue().value())}; // This automatically makes it agnostic to Degrees/Radians/Turns (By making it a Rotation2d)
        state = frc::SwerveModuleState::Optimize(state, current); // Optimizes using the Rotation2d of where it wants to go and where it's at right now

        double target_s = (state.angle.Degrees().value() / 360); // Converts the degrees to turns (0-1) by dividing by 360
        double target_d = (state.speed.value() / wheel_c) * ratio_d; // This converts from m/s to rotation speed. Figure out ratio_d and it should work pretty good

        motor_s.Set(pid_s.Calculate(status_c.GetValue().value() * ratio_s, target_s)); // Takes the current CANCoder rotation and uses it to move to the target rotation
        motor_d.Set(pid_d.Calculate(encode_d.GetVelocity(), target_d)); // Takes the current Velocity and uses it to reach the target speed
    }
};
// =====================================================================================
// The Actual Robot stuff is down here
// =====================================================================================
class Robot : public frc::TimedRobot {

    const double ratio_s = 1; // Relative Motor Counts per Steer Rotation. This is 1 for CANCoder, and 360 with Relative Steering Encoder (I have no idea why)
    const double ratio_d = std::numbers::pi; // Relative motor counts per Drive rotation (TODO: Check this, cuz this ain't right, no way)


    const double wheel_d = 0.1016; // Wheel Diameter (m)
    const double wheel_c = wheel_d * std::numbers::pi; // Wheel Circumference 

    const double robot_r = std::sqrt((0.2889 * 0.2889) + (0.2635 * 0.2635)); // Radius of the robot to it's wheels from it's center
    const double robot_c = robot_r * 2 * std::numbers::pi; // Circumfrence of the robot's radius
    // PIDs
    frc::PIDController // (I put a line break here so they'd be closer together)
    launcherPID{.2, 0, 0.02}; // Launcher PID
    double sP = 0.2, sI = 0, sD = 0; // Steer PID
    double dP = 0.2, dI = 0, dD = 0; // Drive PID
    
    const double max_drive = (wheel_c * 6784)/(60*14.5);//4.46; // Max drive speed of the robot (not motor) in (m/s)
    const double max_rotate =((2 * (std::numbers::pi)) * max_drive) / robot_c; // Max rotate speed of the robot (not motor) in radians/s)
    const double launchSpeed = 2.85; // DON'T TOUCH THIS. The math doesn't make sense but it works as is so just DO. NOT. MESS. WITH. IT.

    // LimeLight Stuff
    double tx = LimelightHelpers::getTX("");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers::getTY("");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers::getTA(""); // Target area (0% to 100% of image)

    // Controller - Only one, cuz we're poor ): 
    frc::XboxController controller_0{0};

    // Swerve wheel location Offsets
    frc::Translation2d fl{ 0.2889_m,  0.2635_m}; 
    frc::Translation2d fr{ 0.2889_m, -0.2635_m};
    frc::Translation2d bl{-0.2889_m,  0.2635_m};
    frc::Translation2d br{-0.2889_m, -0.2635_m};

    // Sets the kinematics using the wheel offsets
    frc::SwerveDriveKinematics<4> kinematics{fl, fr, bl, br};

    // Initializing the swerve modules in the class
    SwerveModule module_4{4, sP, sI, sD, dP, dI, dD};
    SwerveModule module_3{3, sP, sI, sD, dP, dI, dD};
    SwerveModule module_1{1, sP, sI, sD, dP, dI, dD};
    SwerveModule module_2{2, sP, sI, sD, dP, dI, dD};

    // Non-drivetrain Stuff
    bool intakeOn = false;
    bool launcherOn = false;
    frc::Spark intakeMotor{0}; // Intake Motor | on PWM 0
    frc::Spark ropeMotor{2}; // Rope Motor | on PWM 2 (Will probably get deleted, cuz the rope aint working)
    frc::Spark uptakeMotor{3}; // Uptake Motor | on PWM 3
    frc::Spark launchMotor{1}; // Launch Motor | on PWM 1
    frc::Encoder encode_l1{0, 1, false, frc::Encoder::EncodingType::k2X}; // Launch Encoder, using DIO slots 0 and 1 (Blue and Yellow)

    void drive(double vx, double vy, double omega) {
        // Refreshes the Cancoders
        module_4.RefreshCancoder();
        module_3.RefreshCancoder();
        module_1.RefreshCancoder();
        module_2.RefreshCancoder();

        // If the controller is giving 0, don't move any motors
        if (std::abs(vx) < 0.001 && std::abs(vy) < 0.001 && std::abs(omega) < 0.001) {
            module_4.Stop();
            module_3.Stop();
            module_1.Stop();
            module_2.Stop();
            return;
        }
        // Speeds it's setting before giving them to kinematics
        frc::ChassisSpeeds speeds{
            units::meters_per_second_t(vx),
            units::meters_per_second_t(vy),
            units::radians_per_second_t(omega)
        };

        auto states = kinematics.ToSwerveModuleStates(speeds); // Giving them to kinematics
        frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&states, units::meters_per_second_t(max_drive));

        module_4.Set(states[0], ratio_s, ratio_d, wheel_c);
        module_3.Set(states[1], ratio_s, ratio_d, wheel_c);
        module_1.Set(states[2], ratio_s, ratio_d, wheel_c);
        module_2.Set(states[3], ratio_s, ratio_d, wheel_c);
    }
public:
    Robot() {


    }
    void TeleopInit() override {
      // Syncs all the Relative Encoders to their CANcoders
        module_4.SyncEncoderToCancoder(ratio_s);
        module_3.SyncEncoderToCancoder(ratio_s);
        module_1.SyncEncoderToCancoder(ratio_s);
        module_2.SyncEncoderToCancoder(ratio_s);
    }
    void TeleopPeriodic() override {
        drive(
          frc::ApplyDeadband(controller_0.GetLeftY(),  0.08) * max_drive, // Left Stick | Y Axis (For Translation) 
          frc::ApplyDeadband(controller_0.GetLeftX(),  0.08) * max_drive, // Left Stick | X Axis (For Translation) 
          frc::ApplyDeadband(controller_0.GetRightX(), 0.08) * max_rotate // Right Stick | X Axis (For Rotation) 
        ); // The deadband just means, "If the value is below this, set it to 0" (Gets rid of noise)

        /*
        These are all just controls, here's the control scheme:
        A: Rope Motor
        B: Sends the uptake down
        X: Toggles intake
        Y: Reverses intake (I think?)

        Left Bumper: Toggles launcher
        Right Bumper: Sends uptake up
        
        Just like, turn on the launcher throughout the match and use the right bumper to actually launch fuel
        */ 
      
        if (controller_0.GetXButtonPressed()) intakeOn = !intakeOn;

        if (intakeOn) {
          intakeMotor.Set(-0.5);
        }else {
          intakeMotor.Set(0.0);
          uptakeMotor.Set(0.0);
        }
        // Uptake Forward
        if (controller_0.GetRightBumperButton())         uptakeMotor.Set(-0.6);
        if (controller_0.GetRightBumperButtonReleased())  uptakeMotor.Set(0);
        // Uptake Backward
        if (controller_0.GetBButton())                    uptakeMotor.Set(0.6);
        if (controller_0.GetBButtonReleased())            uptakeMotor.Set(0);
        // Launcher Toggles
        if (controller_0.GetLeftBumperButtonPressed()) launcherOn = !launcherOn;
        if (launcherOn) launchMotor.Set(-launcherPID.Calculate(-((encode_l1.GetRate() / 2048) / 100), launchSpeed));
        else launchMotor.Set(0);
        // Rope motor activate
        if (controller_0.GetAButtonPressed())  ropeMotor.Set(1);
        if (controller_0.GetAButtonReleased()) ropeMotor.Set(0);
        // Intake Reverse
        if (controller_0.GetYButton())         intakeMotor.Set(-1);
        if (controller_0.GetYButtonReleased()) intakeMotor.Set(0);

        // Resets PIDs
        if (controller_0.GetStartButtonPressed()) {
            module_4.ResetPIDs();
            module_3.ResetPIDs();
            module_1.ResetPIDs();
            module_2.ResetPIDs();
        }
    }
    void RobotPeriodic() override {
      // Wraps the Encoders, but again we likely don't need to do this
        module_4.WrapEncoder();
        module_3.WrapEncoder();
        module_1.WrapEncoder();
        module_2.WrapEncoder();
    }
    void RobotInit() override { // TODO: If this works code auto aiming.
      cs::HttpCamera limelight{"Limelight","http://limelight.local:5800/stream.mjpg"};
      frc::CameraServer::StartAutomaticCapture(limelight);
    }
private:
};
#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif