// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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
#include <frc/controller/BangBangController.h>
#include <frc/Encoder.h>
#include <LimelightHelpers.h>
#include <cameraserver/CameraServer.h>
#include <cscore.h>
#include <cscore_oo.h>

 // Motor is 1 count / revolution, was 409
using namespace units::literals;

class Robot : public frc::TimedRobot {
  
  // Gear Ratios
  
  //const double ratio_s = 21.42857142857143;
  //const double ratio_s = 150.0 / 7.0;
  const double bang = .9;
  const double ratio_s = 360;
  // const double ratio_d = 8.14;
  const double ratio_d = std::numbers::pi;

  
  // Wheel Size
  const double wheel_d = 0.1016;
  const double wheel_c = wheel_d * std::numbers::pi;
  // PID Constants
  double sP = 0.15;
  double sI = 0;
  double sD = 0;
  double dP = 0.2;
  double dI = 0;
  double dD = 0;
  // Max Speed
  const double max_drive = 4.4196; // m/s
  const double max_rotate = 710 / (150 / 7); // rad/s
  // Kinematics
  frc::Translation2d fl{0.2889_m,  0.2635_m}; //4 
  frc::Translation2d fr{0.2889_m, -0.2635_m}; //3
  frc::Translation2d bl{-0.2889_m, 0.2635_m}; //1
  frc::Translation2d br{-0.2889_m,-0.2635_m}; //2

  frc::SwerveDriveKinematics<4> kinematics{fl, fr, bl, br};
  // Controller
  frc::XboxController controller_0{0};
  
  // CANCoders
  ctre::phoenix6::hardware::CANcoder cancoder_4{43};
  ctre::phoenix6::hardware::CANcoder cancoder_3{33};
  ctre::phoenix6::hardware::CANcoder cancoder_1{13};
  ctre::phoenix6::hardware::CANcoder cancoder_2{23};

  ctre::phoenix6::StatusSignal<units::angle::turn_t> status_c4 = cancoder_4.GetAbsolutePosition();
  ctre::phoenix6::StatusSignal<units::angle::turn_t> status_c3 = cancoder_3.GetAbsolutePosition();
  ctre::phoenix6::StatusSignal<units::angle::turn_t> status_c1 = cancoder_1.GetAbsolutePosition();
  ctre::phoenix6::StatusSignal<units::angle::turn_t> status_c2 = cancoder_2.GetAbsolutePosition();

  // Non-DriveTrain motors
  frc::Spark intakeMotor{0};
  frc::Spark ropeMotor{2};
  frc::Spark uptakeMotor{3};
  frc::Spark launchMotor{1};

  // frc::BangBangController launcherBang;
  frc::PIDController launcherBang{0.5, 0, 0};
  frc::Encoder encode_l1{0, 1, false, frc::Encoder::EncodingType::k2X};
  // Blue 0
  // Yellow 1
  // Green 2
  // White 3

  bool intakeOn = false;
  bool launcherOn = false;
  double launcherGoal = 0;
  // Steer Motors + PID
  rev::spark::SparkFlex motor_s4{41, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex motor_s3{31, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex motor_s1{11, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex motor_s2{21, rev::spark::SparkLowLevel::MotorType::kBrushless};

  frc::PIDController pid_s4{sP, sI, sD};
  frc::PIDController pid_s3{sP, sI, sD};
  frc::PIDController pid_s1{sP, sI, sD};
  frc::PIDController pid_s2{sP, sI, sD};

  rev::spark::SparkRelativeEncoder encode_s4 = motor_s4.GetEncoder();
  rev::spark::SparkRelativeEncoder encode_s3 = motor_s3.GetEncoder();
  rev::spark::SparkRelativeEncoder encode_s1 = motor_s1.GetEncoder();
  rev::spark::SparkRelativeEncoder encode_s2 = motor_s2.GetEncoder();

 // Drive Motors + PID
  rev::spark::SparkFlex motor_d4{42, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex motor_d3{32, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex motor_d1{12, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex motor_d2{22, rev::spark::SparkLowLevel::MotorType::kBrushless};

  frc::PIDController pid_d4{dP, dI, dD};
  frc::PIDController pid_d3{dP, dI, dD};
  frc::PIDController pid_d1{dP, dI, dD};
  frc::PIDController pid_d2{dP, dI, dD};

  rev::spark::SparkRelativeEncoder encode_d4 = motor_d4.GetEncoder();
  rev::spark::SparkRelativeEncoder encode_d3 = motor_d3.GetEncoder();
  rev::spark::SparkRelativeEncoder encode_d1 = motor_d1.GetEncoder();
  rev::spark::SparkRelativeEncoder encode_d2 = motor_d2.GetEncoder();

void drive(double vx, double vy, double omega) {
    status_c4.Refresh();
    status_c3.Refresh();
    status_c1.Refresh();
    status_c2.Refresh();
  if (std::abs(vx) < 0.001 && std::abs(vy) < 0.001 && std::abs(omega) < 0.001) {
    motor_d4.Set(0);
    motor_d3.Set(0);
    motor_d1.Set(0);
    motor_d2.Set(0);
    
    motor_s4.Set(0);
    motor_s3.Set(0);
    motor_s1.Set(0);
    motor_s2.Set(0);
    return;
  }
  frc::ChassisSpeeds speeds{
    units::meters_per_second_t(vx),
    units::meters_per_second_t(vy),
    units::radians_per_second_t(omega)
  };

  auto states = kinematics.ToSwerveModuleStates(speeds);

  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
      &states, units::meters_per_second_t(max_drive));

  frc::Rotation2d current_s4{units::turn_t(status_c4.GetValue().value() / ratio_s)};
  frc::Rotation2d current_s3{units::turn_t(status_c3.GetValue().value() / ratio_s)};
  frc::Rotation2d current_s1{units::turn_t(status_c1.GetValue().value() / ratio_s)};
  frc::Rotation2d current_s2{units::turn_t(status_c2.GetValue().value() / ratio_s)};
  states[0] = frc::SwerveModuleState::Optimize(states[0], current_s4);
  states[1] = frc::SwerveModuleState::Optimize(states[1], current_s3);
  states[2] = frc::SwerveModuleState::Optimize(states[2], current_s1);
  states[3] = frc::SwerveModuleState::Optimize(states[3], current_s2);

  double target_s4 = (states[0].angle.Degrees().value() / 360);
  double target_s3 = (states[1].angle.Degrees().value() / 360);
  double target_s1 = (states[2].angle.Degrees().value() / 360);
  double target_s2 = (states[3].angle.Degrees().value() / 360);

  double target_d4 = (states[0].speed.value() / wheel_c) * ratio_d;
  double target_d3 = (states[1].speed.value() / wheel_c) * ratio_d;
  double target_d1 = (states[2].speed.value() / wheel_c) * ratio_d;
  double target_d2 = (states[3].speed.value() / wheel_c) * ratio_d;

  motor_s4.Set(pid_s4.Calculate(status_c4.GetValue().value(), target_s4));
  motor_s3.Set(pid_s3.Calculate(status_c3.GetValue().value(), target_s3));
  motor_s1.Set(pid_s1.Calculate(status_c1.GetValue().value(), target_s1));
  motor_s2.Set(pid_s2.Calculate(status_c2.GetValue().value(), target_s2));

  motor_d4.Set(pid_d4.Calculate(encode_d4.GetVelocity(), target_d4));
  motor_d3.Set(pid_d3.Calculate(encode_d3.GetVelocity(), target_d3));
  motor_d1.Set(pid_d1.Calculate(encode_d1.GetVelocity(), target_d1));
  motor_d2.Set(pid_d2.Calculate(encode_d2.GetVelocity(), target_d2));

  frc::SmartDashboard::PutNumber("drive velocity (2)", encode_d2.GetVelocity());

  frc::SmartDashboard::PutNumber("Corner 3 PID", std::clamp(pid_s4.Calculate(status_c4.GetValue().value(), target_s4), -1.0, 1.0));
  frc::SmartDashboard::PutNumber("Corner 3 pid", std::clamp(pid_s3.Calculate(status_c3.GetValue().value(), target_s3), -1.0, 1.0));
  frc::SmartDashboard::PutNumber("Corner 1 pid", std::clamp(pid_s1.Calculate(status_c1.GetValue().value(), target_s1), -1.0, 1.0));
  frc::SmartDashboard::PutNumber("Corner 2 pid", std::clamp(pid_s2.Calculate(status_c2.GetValue().value(), target_s2), -1.0, 1.0));
    
  frc::SmartDashboard::PutNumber("Steer 1 Goal", target_s1);
  frc::SmartDashboard::PutNumber("Steer 2 Goal", target_s2);
  frc::SmartDashboard::PutNumber("Steer 3 Goal", target_s3);
  frc::SmartDashboard::PutNumber("Steer 4 Goal", target_s4);
    
  frc::SmartDashboard::PutNumber("Drive 1 Goal", target_d1);
  frc::SmartDashboard::PutNumber("Drive 2 Goal", target_d2);
  frc::SmartDashboard::PutNumber("Drive 3 Goal", target_d3);
  frc::SmartDashboard::PutNumber("Drive 4 Goal", target_d4);
  
  frc::SmartDashboard::PutNumber("Drive 4 e", encode_d4.GetVelocity());
}


 public:
  
  void TeleopInit() override {
  // Reset Cancoders
    status_c4.Refresh();
    status_c3.Refresh();
    status_c1.Refresh();
    status_c2.Refresh();
    encode_s4.SetPosition(status_c4.GetValue().value() * ratio_s);
    encode_s3.SetPosition(status_c3.GetValue().value() * ratio_s);
    encode_s1.SetPosition(status_c1.GetValue().value() * ratio_s);
    encode_s2.SetPosition(status_c2.GetValue().value() * ratio_s);

  }
  void TeleopPeriodic() override {
    status_c4.Refresh();
    status_c3.Refresh();
    status_c1.Refresh();
    status_c2.Refresh();
    if (controller_0.GetXButtonPressed()){
      intakeOn = !intakeOn;
    }

    if (intakeOn) {
      intakeMotor.Set(-0.5);

    } else {
      intakeMotor.Set(0.0);
      uptakeMotor.Set(0.0);
    }


    if (controller_0.GetRightBumperButton()){
      uptakeMotor.Set(-0.6);

    }
     if (controller_0.GetRightBumperButtonReleased()){
      uptakeMotor.Set(0);
    }
    if (controller_0.GetBButton()){
      uptakeMotor.Set(0.6);

    }
     if (controller_0.GetBButtonReleased()){
      uptakeMotor.Set(0);
      
    }
     if (controller_0.GetLeftBumperButtonPressed()){
      launcherOn = !launcherOn;
    }

    if (launcherOn) {
      launcherGoal = (-launcherBang.Calculate(-((encode_l1.GetRate() / 8192) / 20), bang));
      launchMotor.Set(launcherGoal);

      frc::SmartDashboard::PutNumber("getrate", (-((encode_l1.GetRate() / 8192) / 20)));
      frc::SmartDashboard::PutNumber("launcherbang", launcherGoal);
      
    } else {
      launchMotor.Set(0);
    }


    if (controller_0.GetAButtonPressed()){
      ropeMotor.Set(1);
    }
     if (controller_0.GetAButtonReleased()){
      ropeMotor.Set(0);
    }

    if (controller_0.GetYButton()){
      intakeMotor.Set(-1);
    }
     if (controller_0.GetYButtonReleased()){
      intakeMotor.Set(0);
    }

    if (controller_0.GetStartButtonPressed()){
      pid_s1.Reset();
      pid_s2.Reset();
      pid_s3.Reset();
      pid_s4.Reset();
      pid_d1.Reset();
      pid_d2.Reset();
      pid_d3.Reset();
      pid_d4.Reset();
    }

    

  // Drive Function
  drive(
    (frc::ApplyDeadband(controller_0.GetLeftY(), 0.08) * max_drive), 
    (frc::ApplyDeadband(controller_0.GetLeftX(), 0.08) * max_drive), 
    (frc::ApplyDeadband(controller_0.GetRightX(), 0.08) * max_rotate));

  frc::SmartDashboard::PutNumber("Left X", controller_0.GetLeftX());
  frc::SmartDashboard::PutNumber("Left Y", controller_0.GetLeftY());
  frc::SmartDashboard::PutNumber("Right X", controller_0.GetRightX());
  }
  void RobotPeriodic() override {

  if (encode_s4.GetPosition() > 360) {encode_s4.SetPosition(encode_s4.GetPosition() - 360);}
  if (encode_s3.GetPosition() > 360) {encode_s3.SetPosition(encode_s3.GetPosition() - 360);}
  if (encode_s1.GetPosition() > 360) {encode_s1.SetPosition(encode_s1.GetPosition() - 360);}
  if (encode_s2.GetPosition() > 360) {encode_s2.SetPosition(encode_s2.GetPosition() - 360);}

  if (encode_s4.GetPosition() < 0) {encode_s4.SetPosition(encode_s4.GetPosition() + 360);}
  if (encode_s3.GetPosition() < 0) {encode_s3.SetPosition(encode_s3.GetPosition() + 360);}
  if (encode_s1.GetPosition() < 0) {encode_s1.SetPosition(encode_s1.GetPosition() + 360);}
  if (encode_s2.GetPosition() < 0) {encode_s2.SetPosition(encode_s2.GetPosition() + 360);}


  // Encoder locations to show on Shuffleboard
  frc::SmartDashboard::PutNumber("S4 Relative", encode_s4.GetPosition());
  frc::SmartDashboard::PutNumber("S3 Relative", encode_s3.GetPosition());
  frc::SmartDashboard::PutNumber("S1 Relative", encode_s1.GetPosition());
  frc::SmartDashboard::PutNumber("S2 Relative", encode_s2.GetPosition());
  
  frc::SmartDashboard::PutNumber("drive encoder", encode_d3.GetPosition());
  status_c4.Refresh();
  status_c3.Refresh();
  status_c1.Refresh();
  status_c2.Refresh();
  frc::SmartDashboard::PutNumber("C4 Absolute", status_c4.GetValue().value());
  frc::SmartDashboard::PutNumber("C3 Absolute", status_c3.GetValue().value());
  frc::SmartDashboard::PutNumber("C1 Absolute", status_c1.GetValue().value());
  frc::SmartDashboard::PutNumber("C2 Absolute", status_c2.GetValue().value());
}
  void RobotInit() override {
//   frc::CameraServer::StartAutomaticCapture(
//     cs::HttpCamerall_camera("limelight", "http://limelight-homer.local:5800/stream.mjpg", cs::HttpCamera::HttpCameraKind::kMJPGStreamer);
//   )
//   ll_camera.SetResolution(320, 240);
//   ll_camera.SetFPS(30);   
//   cs::CvSink cvSink("vision_sink");
//   cvSink.SetSource(ll_camera);
//   cvSink.SetEnabled(true);
//   cv::Mat frame;
// std::uint64_t frame_time = cvSink.GrabFrame(frame);
// if (!frame_time) {
//     fmt::print("Error grabbing frame: {}\n", cvSink.GetError());
// } else {
// }
// cs::CvSource outputStream = frc::CameraServer::PutVideo("Processed", 320, 240);
// outputStream.PutFrame(frame);
  
    cs::HttpCamera limelight{"Limelight","http://limelight.local:5800/stream.mjpg"};
    frc::CameraServer::StartAutomaticCapture(limelight);
  }
  

  Robot() {
    pid_s4.EnableContinuousInput(0, 1);
    pid_s3.EnableContinuousInput(0, 1);
    pid_s1.EnableContinuousInput(0, 1);
    pid_s2.EnableContinuousInput(0, 1);
  }
 private:
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
