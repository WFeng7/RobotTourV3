// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

#include <units/velocity.h>


using namespace pathplanner;
Drivetrain::Drivetrain() {
    
    rightSpark1.SetInverted(1); 
    rightSpark2.SetInverted(1);// inverts the right drive motors
    resetGyro();


    AutoBuilder::configureRamsete(
    [this]() -> frc::Pose2d { return getPose(); },              // Lambda for Robot pose supplier
    [this](frc::Pose2d pose) { resetPose(pose); },              // Lambda for resetting odometry
    [this]() -> frc::ChassisSpeeds { return getRobotRelativeSpeeds(); },  // Lambda for ChassisSpeeds supplier (MUST BE ROBOT RELATIVE)
    [this](frc::ChassisSpeeds speeds) { driveRobotRelative(speeds); },   // Lambda for driving the robot given ROBOT RELATIVE ChassisSpeeds
    ReplanningConfig(),                                         // Default path replanning config
    [this]() -> bool { /* Implement shouldFlipPath logic here */ return true; },  // Lambda for shouldFlipPath
    this                                                        // Reference to this subsystem to set requirements
);


    
}

void Drivetrain::setDriveMotors(double left, double right) {
    //tab.Add("Left Motor", left);
    //tab.Add("Right Motor", right);
    
    if (flipped) {
        leftSpark1.Set(-right);
        leftSpark2.Set(-right);
        rightSpark1.Set(-left);
        rightSpark2.Set(-left);
    } else {
        leftSpark1.Set(left);
        leftSpark2.Set(left);
        rightSpark1.Set(right);
        rightSpark2.Set(right);
    }
    //NetworkTableEntry testTab = Shuffleboard.getTab("Test").add("Pi", 3.14);
}

void Drivetrain::arcadeDrive(double throttle, double turn) {
    setDriveMotors(throttle + turn, throttle - turn);
}

void Drivetrain::stop() {
    leftSpark1.Set(0);
    leftSpark2.Set(0);
    rightSpark1.Set(0);
    rightSpark2.Set(0);
}

void Drivetrain::flipDT() {
    flipped = !flipped;
}

frc::Rotation2d Drivetrain::getHeading() {
    units::degree_t deg{-gyro.GetAngle()};
    return frc::Rotation2d(deg);
}

double Drivetrain::getHeadingAsAngle() {
    return getHeading().Degrees().value();
}

frc::Rotation2d Drivetrain::getPitch() {
    units::degree_t deg{gyro.GetPitch()};
    return frc::Rotation2d(deg);
}

frc::Pose2d Drivetrain::getPose(){
    units::meter_t posX{gyro.GetDisplacementX()};
    units::meter_t posY{gyro.GetDisplacementY()};
    units::degree_t deg{gyro.GetPitch()};
    return frc::Pose2d(posX, posY, deg);
}


frc::Pose2d Drivetrain::resetPose(frc::Pose2d pose){
    return frc::Pose2d();
}

frc::ChassisSpeeds Drivetrain::getRobotRelativeSpeeds(){

    units::meters_per_second_t speedX{gyro.GetVelocityX()};
    units::meters_per_second_t speedY{gyro.GetVelocityY()};

    frc::ChassisSpeeds speeds{speedX, -speedY,
    units::radians_per_second_t(std::numbers::pi)};
    return frc::ChassisSpeeds(speeds);
}

frc::DifferentialDriveWheelSpeeds Drivetrain::driveRobotRelative(frc::ChassisSpeeds speeds){
    // Creating my kinematics object: track width of 23 inches (Calculated from Robot CAD)
    frc::DifferentialDriveKinematics kinematics{23_in};

// Convert to wheel speeds. Here, we can use C++17's structured bindings
// feature to automatically split the DifferentialDriveWheelSpeeds
// struct into left and right velocities.
    auto [left,right] = kinematics.ToWheelSpeeds(speeds);
    return frc::DifferentialDriveWheelSpeeds(left, right);

}



double Drivetrain::getPitchAsAngle() {
    return getPitch().Degrees().value();
}



void Drivetrain::resetEncoders() {
    leftEncoder.SetPosition(0.0);
    rightEncoder.SetPosition(0.0);
}

double Drivetrain::getLeftEncoderDistance() {
    return leftEncoder.GetPosition();
}

double Drivetrain::getRightEncoderDistance() {
    return rightEncoder.GetPosition();
}

double Drivetrain::neoTicksToInches(double revolutions) {
    return M_PI * drivetrain::wheelDiameter * revolutions / 10.71;
}

void Drivetrain::resetGyro() {
    gyro.Reset();
}


// This method will be called once per scheduler run
void Drivetrain::Periodic() {}

