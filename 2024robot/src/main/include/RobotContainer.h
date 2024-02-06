// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/JoystickButton.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"


#include "subsystems/Drivetrain.h"


#include "commands/SlowAuto.h"
#include "commands/ArcadeDrive.h"
#include "commands/FlipDrivetrain.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;

  //drivetrain
  Drivetrain m_drivetrain;

  //

   // chooser for autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;
  SlowAuto m_slowauto{&m_drivetrain};
  std::unique_ptr<frc2::Command> exampleAuto;
  std::unique_ptr<frc2::Command> pieceAuto;

  //joystick
  frc::Joystick m_joystick{oi::kDriveJoystickPort};
  frc::Joystick m_joystickArm{oi::kArmJoystickPort};


  void ConfigureButtonBindings();

  
};
