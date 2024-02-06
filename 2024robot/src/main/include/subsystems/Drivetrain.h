// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

#include <frc2/command/SubsystemBase.h>

#include <CANVenom.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkLowLevel.h>
#include <rev/CANSparkBase.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

#include <frc/SPI.h>
#include <AHRS.h>

#include <frc/geometry/Rotation2d.h>

#include <frc/drive/DifferentialDrive.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

#include "Constants.h"

using namespace pathplanner;

class Drivetrain : public frc2::SubsystemBase {
  
  
 public:
  Drivetrain();

  void setDriveMotors(double, double);

  void arcadeDrive(double, double);

  void stop();
  
  void flipDT();

  frc::Rotation2d getHeading();

  double getHeadingAsAngle();

  void resetEncoders();

  frc::Rotation2d getPitch();

  double getPitchAsAngle();

  double getLeftEncoderDistance();

  double getRightEncoderDistance();

  double neoTicksToInches(double);

  void resetGyro();
  
  frc::Pose2d getPose();

  frc::Pose2d resetPose(frc::Pose2d);

  frc::ChassisSpeeds getRobotRelativeSpeeds();

  frc::DifferentialDriveWheelSpeeds driveRobotRelative(frc::ChassisSpeeds);



  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

  //ctre::phoenix::motorcontrol::can::WPI_VictorSPX leftSpark1{drivetrain::kLeftDriveSparkPort1};
  //frc::CANVenom leftSpark2{drivetrain::kLeftDriveSparkPort2};

  rev::CANSparkMax leftSpark1{drivetrain::kLeftDriveSparkPort1, rev::CANSparkLowLevel::MotorType::kBrushless}; // defining left side spark motor 1
  rev::CANSparkMax leftSpark2{drivetrain::kLeftDriveSparkPort2, rev::CANSparkLowLevel::MotorType::kBrushless}; // defining left side spark motor 2

  //ctre::phoenix::motorcontrol::can::WPI_VictorSPX rightSpark1{drivetrain::kLeftDriveSparkPort1};
  //frc::CANVenom rightSpark2{drivetrain::kRightDriveSparkPort2};

  rev::CANSparkMax rightSpark1{drivetrain::kLeftDriveSparkPort1, rev::CANSparkLowLevel::MotorType::kBrushless}; // defining right side motor 1
  rev::CANSparkMax rightSpark2{drivetrain::kLeftDriveSparkPort2, rev::CANSparkLowLevel::MotorType::kBrushless}; // defining right side motor 2


  //frc::MotorControllerGroup::MotorControllerGroup leftMotors{rightSpark1, leftSpark1}; // assigning left side motors into one group
  //frc::MotorControllerGroup::MotorControllerGroup rightMotors{rightSpark2, leftSpark2}; // assigning right side motors into one group
  

  rev::SparkRelativeEncoder leftEncoder = leftSpark1.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096);
  rev::SparkRelativeEncoder rightEncoder = rightSpark1.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096);
  //frc::DifferentialDrive drive{leftMotors, rightMotors};

  AHRS gyro{frc::SPI::Port::kMXP};

  bool flipped = false;

  

  frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("Test");

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  

  
};






