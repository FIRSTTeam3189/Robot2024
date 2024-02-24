// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterAutoAlign.h"

ShooterAutoAlign::ShooterAutoAlign(Shooter* shooter, PoseEstimatorHelper *estimator, Vision *vision) :
m_shooter(shooter),
m_estimator(estimator),
m_vision(vision) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
  AddRequirements(estimator);
  AddRequirements(vision);
}

// Called when the command is initially scheduled.
void ShooterAutoAlign::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShooterAutoAlign::Execute() {
  auto angle = CalculateShooterAngle();
  m_shooter->SetRotation(angle);
}

// Called once the command ends or is interrupted.
void ShooterAutoAlign::End(bool interrupted) {}

// Returns true when the command should end.
bool ShooterAutoAlign::IsFinished() {
  return false;
}

units::degree_t ShooterAutoAlign::CalculateShooterAngle() {
  frc::Pose2d currentPose = m_estimator->GetEstimatedPose();
  frc::Pose3d tagPose = frc::Pose3d{};
  // auto autoData = m_vision->GetVisionData();
  // Set to false instead of isDetected to use FMS for alliance deduction instead
  // bool isDetected = false;

  // If vision detects a tag, determine what side we're on (1-5 for red, 6-10 for blue)
  // Based on this side, we use the corresponding speaker pose
  // If vision is not detecting (or isDetected is set to false for testing)
  // Then we get the alliance side from FMS and use that instead
  // if (isDetected) {
  //   // Red, use #4
  //   if (autoData.ID < 6) {
  //     tagPose = VisionConstants::kTagPoses.at(3);
  //   } else if (autoData.ID > 5 && autoData.ID < 11) { // Blue, use #7
  //     tagPose = VisionConstants::kTagPoses.at(6);
  //   }
  // } else {
  auto allianceSide = frc::DriverStation::GetAlliance().value();
  if (allianceSide == frc::DriverStation::Alliance::kRed) {
    tagPose = VisionConstants::kTagPoses.at(3);
  } else {
    tagPose = VisionConstants::kTagPoses.at(6);
  }
  // }

  // Use inverse tangent of height over distance to calculate shooter angle
  double distance = sqrt(pow((abs(tagPose.X().value() - currentPose.X().value())), 2.0) + 
                         pow((tagPose.Y().value() - currentPose.Y().value()), 2.0));

  // Subtract distance from shooting axle to center of robot since pose is from center
  distance -= ShooterConstants::kAxleToCenterDistance.value();

  // Subtract distance from axle to ground since we fire from axle not from ground which the speaker height is measured from
  double height = ShooterConstants::kSpeakerHeightTarget.value() - ShooterConstants::kAxleToGroundDistance.value();
  double angle = atan(height / distance);
  frc::SmartDashboard::PutNumber("Shooter auto align distance", distance);
  frc::SmartDashboard::PutNumber("Shooter auto align height", height);
  frc::SmartDashboard::PutNumber("Shooter auto align angle", units::degree_t{units::radian_t{angle}}.value());
  return units::degree_t{units::radian_t{angle}};
}