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
}

// Called when the command is initially scheduled.
void ShooterAutoAlign::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShooterAutoAlign::Execute() {
  frc::Pose2d currentPose = m_estimator->GetEstimatedPose();
  auto autoData = m_vision->GetVisionData();
  bool isDetected = autoData.isDetected;
  // If vision detects a tag, determine what side we're on (1-5 for red, 6-10 for blue)
  // Based on this side, we use the corresponding speaker pose
  // If vision is not detecting (or isDetected is set to false for testing)
  // Then we get the alliance side from FMS and use that instead
  if (isDetected) {
    // Red
    if (autoData.ID < 6) {
      
    } else if (autoData.ID > 5 && autoData.ID < 11) { // Blue

    }
  } else {
    auto allianceSide = frc::DriverStation::GetAlliance();
    if (allianceSide == frc::DriverStation::Alliance::kRed) {

    } else {

    }
  }
}

// Called once the command ends or is interrupted.
void ShooterAutoAlign::End(bool interrupted) {
  m_shooter->StopRotation();
}

// Returns true when the command should end.
bool ShooterAutoAlign::IsFinished() {
  return false;
}
