// #define __SwerveTest__ 


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
#include <vector>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/Shuffleboard.h>

using namespace units::literals;
// Groups the swerve variables for outputting on Elastic
struct SwerveStruct {
  public: 
  double target_s_output;
  double target_d_output;
  double pid_s_target_output;
  double pid_d_target_output;
  double drive_encoder_count;
  double drive_encoder_velocity;
  double cancoder_position;
};
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
    void ReadDiagnostics(SwerveStruct& DataStruct){
        DataStruct.drive_encoder_count = encode_d.GetPosition();
    }
    void RefreshCancoder() { // Refreshes the cancoder for the... dude do I have to specifify it's for the current module you get the idea
        status_c.Refresh();

    } // Syncs the Relative Encoder in the turning Spark to the Absolute in the CANCoder
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
    void Set(frc::SwerveModuleState state, double ratio_s, double ratio_d, double wheel_c, SwerveStruct& DataStruct) {
        status_c.Refresh(); // Refresh the status for the CANcoders to make sure they're updated
        auto current = units::radian_t(status_c.GetValue().value());
        //frc::Rotation2d current{units::turn_t(status_c.GetValue().value())}; // This automatically makes it agnostic to Degrees/Radians/Turns (By making it a Rotation2d)
        auto optimized = frc::SwerveModuleState::Optimize(state, current);
        optimized.speed *= (optimized.angle - current).Cos();
        //auto optimized = frc::SwerveModuleState::Optimize(state, current); // Optimizes using the Rotation2d of where it wants to go and where it's at right now
        double target_s = (optimized.angle.Degrees().value() / 360); // Converts the degrees to turns (0-1) by dividing by 360
        double target_d = (optimized.speed.value() / wheel_c) * ratio_d; // This converts from m/s to rotation speed. Figure out ratio_d and it should work pretty good
       
        double pid_s_target = pid_s.Calculate(status_c.GetValue().value(), target_s); // Takes the current CANCoder rotation and uses it to move to the target rotation
        motor_s.Set(pid_s_target); 

        double pid_d_target = pid_d.Calculate(encode_d.GetVelocity(), target_d); // Takes the current Velocity and uses it to reach the target speed
        motor_d.Set(pid_d_target);
        
        DataStruct.pid_d_target_output = pid_d_target;
        DataStruct.pid_s_target_output = pid_s_target;
        DataStruct.target_d_output = target_d;
        DataStruct.target_s_output = target_s; 
        DataStruct.drive_encoder_velocity = encode_d.GetVelocity();
        DataStruct.cancoder_position = status_c.GetValue().value();


}};
// =====================================================================================
// The Actual Robot stuff is down here
// =====================================================================================
class Robot : public frc::TimedRobot {

    const double ratio_s = 1; // Relative Motor Counts per Steer Rotation. This is 1 for CANCoder, and 360 with Relative Steering Encoder (I have no idea why)
    const double ratio_d = std::numbers::pi / 10; // Relative motor counts per Drive rotation (TODO: Check this, cuz this ain't right, no way)

    const double wheel_d = 0.1016; // Wheel Diameter (m)
    const double wheel_c = wheel_d * std::numbers::pi; // Wheel Circumference 

    const double robot_r = std::sqrt((0.2889 * 0.2889) + (0.2635 * 0.2635)); // Radius of the robot to it's wheels from it's center
    const double robot_c = robot_r * 2 * std::numbers::pi; // Circumfrence of the robot's radius
    
    frc::PIDController launcherPID{.2, 0, 0.02}; // Launcher PID 
    frc::PIDController uptakePID{.2, 0, 0.02}; // Uptake PID 
    frc::PIDController intakePID{.2, 0, 0.02}; // Intake PID 
    frc::PIDController intake2PID{.2, 0, 0.02}; // Intake 2 PID 
  
    double sP = 0.3, sI = 0, sD = 0.03; // Steer PID
    double dP = 0.1, dI = 0, dD = 0; // Drive PID
    
    const double max_drive = (wheel_c * 6784)/(60*14.5);//4.46; // Max drive speed of the robot (not motor) in (m/s)
    const double max_rotate =((2 * (std::numbers::pi)) * max_drive) / robot_c; // Max rotate speed of the robot (not motor) in radians/s)
    const double launchSpeed = 2.85; // DON'T TOUCH THIS. The math doesn't make sense but it works as is so just DO. NOT. MESS. WITH. IT.
    // All of these are in RPS
    const double uptakeSpeed = 5;
    const double uptakeReverseSpeed = -2;
    const double intakeSpeed = 5; 
    const double intakeReverseSpeed = -2;
    const double intake2Speed = 5;
    const double intake2ReverseSpeed = -2; 
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
    SwerveStruct module_4_struct;
    SwerveStruct module_3_struct;
    SwerveStruct module_1_struct;
    SwerveStruct module_2_struct;

    SwerveModule module_4{4, sP, sI, sD, dP, dI, dD};
    SwerveModule module_3{3, sP, sI, sD, dP, dI, dD};
    SwerveModule module_1{1, sP, sI, sD, dP, dI, dD};
    SwerveModule module_2{2, sP, sI, sD, dP, dI, dD};

    // Non-drivetrain Stuff
    bool intakeOn = false;
    bool launcherOn = false;
    frc::Spark intakeMotor{0}; // Intake Motor | on PWM 0
    frc::Spark intakeMotor2{4}; // Intake Motor 2 | on PWM 4
    frc::Spark ropeMotor{2}; // Rope Motor | on PWM 2 (Will probably get deleted, cuz the rope aint working)
    frc::Spark uptakeMotor{3}; // Uptake Motor | on PWM 3
    frc::Spark launchMotor{1}; // Launch Motor | on PWM 1

    frc::Encoder encode_l1{0, 1, false, frc::Encoder::EncodingType::k2X}; // Launch Encoder, using DIO slots 0 and 1 (Blue and Yellow)
    frc::Encoder encode_u1{2, 3, false, frc::Encoder::EncodingType::k2X}; // Uptake Encoder, using DIO slots 2 and 3 (Blue and Yellow)
    frc::Encoder encode_i1{4, 5, false, frc::Encoder::EncodingType::k2X}; // Intake Encoder, using DIO slots 4 and 5 (Blue and Yellow)
    frc::Encoder encode_i2{6, 7, false, frc::Encoder::EncodingType::k2X}; // Intake Encoder, using DIO slots 6 and 7 (Blue and Yellow)

    
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

        module_4.Set(states[0], ratio_s, ratio_d, wheel_c, module_4_struct);
        module_3.Set(states[1], ratio_s, ratio_d, wheel_c, module_3_struct);
        module_1.Set(states[2], ratio_s, ratio_d, wheel_c, module_1_struct);
        module_2.Set(states[3], ratio_s, ratio_d, wheel_c, module_2_struct);
    }
public:
    Robot() {
    }
    void TeleopInit() override {
      module_4.ResetPIDs();
      module_3.ResetPIDs();
      module_1.ResetPIDs();
      module_2.ResetPIDs();
    }
    void TeleopPeriodic() override {
      drive(
        frc::ApplyDeadband(controller_0.GetLeftY(),  0.08) * max_drive, // Left Stick | Y Axis (For Translation) 
        frc::ApplyDeadband(controller_0.GetLeftX(),  0.08) * max_drive, // Left Stick | X Axis (For Translation) 
        frc::ApplyDeadband(controller_0.GetRightX(), 0.08) * max_rotate); // Right Stick | X Axis (For Rotation) 
        
        frc::SmartDashboard::PutNumber("Left X", controller_0.GetLeftX());
        frc::SmartDashboard::PutNumber("Left Y", controller_0.GetLeftY());
        frc::SmartDashboard::PutNumber("Right X", controller_0.GetRightX());
      // The deadband just means, "If the value is below this, set it to 0" (Gets rid of noise)

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
      
      // Intake Toggle
      if (controller_0.GetXButtonPressed()) intakeOn = !intakeOn;
      if (intakeOn) {
        intakeMotor.Set(intakePID.Calculate(((encode_i1.GetRate() / 2048)), intakeSpeed));
        intakeMotor2.Set(intake2PID.Calculate(((encode_i2.GetRate() / 2048)), intake2Speed));
      } else {
        intakeMotor.Set(0.0);
        intakeMotor2.Set(0.0);
      }
      // Intake Reverse
      if (controller_0.GetYButton()) {
        intakeMotor.Set(intakePID.Calculate(((encode_i1.GetRate() / 2048)), intakeReverseSpeed));
        intakeMotor2.Set(intake2PID.Calculate(((encode_i2.GetRate() / 2048)), intake2ReverseSpeed));
      }
      if (controller_0.GetYButtonReleased()) {
        intakeMotor.Set(0);
        intakeMotor2.Set(0);
      }

      // Uptake Forward
      if (controller_0.GetRightBumperButton())         uptakeMotor.Set(uptakePID.Calculate(((encode_u1.GetRate() / 2048)), uptakeSpeed));
      if (controller_0.GetRightBumperButtonReleased())  uptakeMotor.Set(0);

      // Uptake Backward
      if (controller_0.GetBButton())                    uptakeMotor.Set(uptakePID.Calculate(((encode_u1.GetRate() / 2048)), uptakeReverseSpeed));
      if (controller_0.GetBButtonReleased())            uptakeMotor.Set(0);

      // Launcher Toggles
      if (controller_0.GetLeftBumperButtonPressed()) launcherOn = !launcherOn;
      if (launcherOn) launchMotor.Set(-launcherPID.Calculate(-((encode_l1.GetRate() / 2048) / 100), launchSpeed));
      else launchMotor.Set(0);

      // Rope motor activate
      if (controller_0.GetAButtonPressed())  ropeMotor.Set(1);
      if (controller_0.GetAButtonReleased()) ropeMotor.Set(0);

      // Resets PIDs
      if (controller_0.GetStartButtonPressed()) {
        module_4.ResetPIDs();
        module_3.ResetPIDs();
        module_1.ResetPIDs();
        module_2.ResetPIDs();
      }
    }

    void RobotInit() override {
      auto& tab = frc::Shuffleboard::GetTab("Swerve Diagnostic");
      tab.Add("target angle", module_1_struct.pid_s_target_output).WithWidget(frc::BuiltInWidgets::kGraph);
      // Sets the distance/pulse so RPM is returned correctly
      encode_i1.SetDistancePerPulse(1.0 / 2048.0); 
      encode_i2.SetDistancePerPulse(1.0 / 2048.0); 
      encode_u1.SetDistancePerPulse(1.0 / 2048.0);
      // Syncs all the Relative Encoders to their CANcoders
      module_4.SyncEncoderToCancoder(ratio_s);
      module_3.SyncEncoderToCancoder(ratio_s);
      module_1.SyncEncoderToCancoder(ratio_s);
      module_2.SyncEncoderToCancoder(ratio_s);

      cs::HttpCamera limelight{"Limelight","http://limelight.local:5800/stream.mjpg"};
      frc::CameraServer::StartAutomaticCapture(limelight);

      LimelightHelpers::setCameraPose_RobotSpace("",
        -0.0762,    // Forward offset (meters)
        0.1143,    // Side offset (meters)
        0.5334,    // Height offset (meters)
        0.0,    // Roll (degrees)
        0.0,   // Pitch (degrees)          
        0.0     // Yaw (degrees)
      );
    }
    
    void RobotPeriodic() override {

      // Wraps the Encoders, but again we likely don't need to do this
      module_4.WrapEncoder();
      module_3.WrapEncoder();
      module_1.WrapEncoder();
      module_2.WrapEncoder();
      module_1.ReadDiagnostics(module_1_struct);

        frc::SmartDashboard::PutNumber("1 Target S", module_1_struct.target_s_output);
        frc::SmartDashboard::PutNumber("1 Target D", module_1_struct.target_d_output);
        frc::SmartDashboard::PutNumber("1 PID S",    module_1_struct.pid_s_target_output);
        frc::SmartDashboard::PutNumber("1 PID D",    module_1_struct.pid_d_target_output);
        frc::SmartDashboard::PutNumber("1 Drive Encoder", module_1_struct.drive_encoder_count);
        frc::SmartDashboard::PutNumber("1 Drive Encoder Velocity", module_1_struct.drive_encoder_velocity);
        frc::SmartDashboard::PutNumber("1 CANCoder Position", module_1_struct.cancoder_position);

        frc::SmartDashboard::PutNumber("2 Target S", module_2_struct.target_s_output);
        frc::SmartDashboard::PutNumber("2 Target D", module_2_struct.target_d_output);
        frc::SmartDashboard::PutNumber("2 PID S",    module_2_struct.pid_s_target_output);
        frc::SmartDashboard::PutNumber("2 PID D",    module_2_struct.pid_d_target_output);
        frc::SmartDashboard::PutNumber("2 Drive Encoder", module_2_struct.drive_encoder_count);
        frc::SmartDashboard::PutNumber("2 Drive Encoder Velocity", module_2_struct.drive_encoder_velocity);
        frc::SmartDashboard::PutNumber("2 CANCoder Position", module_2_struct.cancoder_position);

        frc::SmartDashboard::PutNumber("3 Target S", module_3_struct.target_s_output);
        frc::SmartDashboard::PutNumber("3 Target D", module_3_struct.target_d_output);
        frc::SmartDashboard::PutNumber("3 PID S",    module_3_struct.pid_s_target_output);
        frc::SmartDashboard::PutNumber("3 PID D",    module_3_struct.pid_d_target_output);
        frc::SmartDashboard::PutNumber("3 Drive Encoder", module_3_struct.drive_encoder_count);
        frc::SmartDashboard::PutNumber("3 Drive Encoder Velocity", module_3_struct.drive_encoder_velocity);
        frc::SmartDashboard::PutNumber("3 CANCoder Position", module_3_struct.cancoder_position);

        frc::SmartDashboard::PutNumber("4 Target S", module_4_struct.target_s_output);
        frc::SmartDashboard::PutNumber("4 Target D", module_4_struct.target_d_output);
        frc::SmartDashboard::PutNumber("4 PID S",    module_4_struct.pid_s_target_output);
        frc::SmartDashboard::PutNumber("4 PID D",    module_4_struct.pid_d_target_output);
        frc::SmartDashboard::PutNumber("4 Drive Encoder", module_4_struct.drive_encoder_count);
        frc::SmartDashboard::PutNumber("4 Drive Encoder Velocity", module_4_struct.drive_encoder_velocity);
        frc::SmartDashboard::PutNumber("4 CANCoder Position", module_4_struct.cancoder_position);
    }
    void AutonomousInit() override {
      module_4.ResetPIDs();
      module_3.ResetPIDs();
      module_1.ResetPIDs();
      module_2.ResetPIDs();
    }
    void AutonomousPeriodic() override { // TODO: See if we have to put a delay on the uptake
      launchMotor.Set(-launcherPID.Calculate(-((encode_l1.GetRate() / 2048) / 100), launchSpeed));
      
      if (launcherPID.AtSetpoint()){ // Turns on the uptake if the launch motor is at the right place
        uptakeMotor.Set(-0.6);
      } else { 
        uptakeMotor.Set(0);
      }
    } 
private:
};
#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif