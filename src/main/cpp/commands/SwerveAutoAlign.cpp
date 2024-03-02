// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SwerveAutoAlign.h"

SwerveAutoAlign::SwerveAutoAlign(SwerveDrive *swerve, bool shouldAlignSpeaker, units::degree_t goal) :
m_swerve(swerve),
m_constraints(SwerveDriveConstants::kMaxAngularVelocity, SwerveDriveConstants::kMaxAngularAcceleration),
m_rotationPIDController(SwerveDriveConstants::kPRot, SwerveDriveConstants::kIRot, SwerveDriveConstants::kDRot, m_constraints),
m_withinThresholdLoops(0),
m_goal(0.0_deg) {
  (void)AutoConstants::kAutonomousPaths[0];
  (void)VisionConstants::kSyncBytes[0];
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve);
  m_rotationPIDController.EnableContinuousInput(0_deg, 360_deg);
  if (shouldAlignSpeaker)
    m_goal = GetSpeakerGoalAngle();
  else
    m_goal = goal;
}

// Called when the command is initially scheduled.
void SwerveAutoAlign::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SwerveAutoAlign::Execute() {
  // Every loop, check if we're at ur goal (within 1 degree) and set to 0 if we aren't
  // After it reaches a certain amount of loops, end the command
  if (fabs(m_swerve->GetNormalizedYaw().value() - m_goal.value()) < AutoConstants::kAutoAlignTolerance)
    m_withinThresholdLoops++;
  else
    m_withinThresholdLoops = 0;

  auto rot = GetDesiredRotationalVelocity();
  m_swerve->Drive(0.0_mps, 0.0_mps, rot, true, frc::Translation2d{});
}

// Called once the command ends or is interrupted.
void SwerveAutoAlign::End(bool interrupted) {}

// Returns true when the command should end.
bool SwerveAutoAlign::IsFinished() {
  return m_withinThresholdLoops >= 10;
}

units::angular_velocity::radians_per_second_t SwerveAutoAlign::GetDesiredRotationalVelocity() {
  // Return next velocity in radians per second as calculated by PIDController and limited by rotLimiter
  units::angular_velocity::radians_per_second_t rot = 
    units::angular_velocity::radians_per_second_t{m_rotationPIDController.Calculate(m_swerve->GetNormalizedYaw(), m_goal)
    * SwerveDriveConstants::kMaxAngularVelocity};

  return rot;
}

units::degree_t SwerveAutoAlign::GetSpeakerGoalAngle() {
  frc::Pose3d tagPose = VisionConstants::kTagPoses.at(6);
  auto allianceSide = frc::DriverStation::GetAlliance();
  if (allianceSide) {
    if (allianceSide.value() == frc::DriverStation::Alliance::kRed) {
      tagPose = VisionConstants::kTagPoses.at(3);
    } else {
      tagPose = VisionConstants::kTagPoses.at(6);
    }
  }

  auto currentPose = m_swerve->GetEstimatedPose();
  
  // Calculate the angle to rotate to for the robot to point towards the speaker
  // This is alliance-dependent 
  auto xDistance = tagPose.X() - currentPose.X();
  auto yDistance = tagPose.Y() - currentPose.Y();
  frc::SmartDashboard::PutNumber("Swerve align x distance", xDistance.value());
  frc::SmartDashboard::PutNumber("Swerve align y distance", yDistance.value());
  // x and y swapped when passed into atan function because our x is their y
  auto goalAngle = units::degree_t{units::radian_t{atan(yDistance.value() / xDistance.value())}};

  if (allianceSide) {
    if (allianceSide.value() == frc::DriverStation::Alliance::kRed) {
        goalAngle += 180.0_deg;
        if (goalAngle > 180.0_deg)
          goalAngle -= 360.0_deg;
    }
  }

  frc::SmartDashboard::PutNumber("Swerve auto align angle", goalAngle.value());
  return goalAngle;
}
