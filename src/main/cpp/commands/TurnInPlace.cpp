
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurnInPlace.h"

TurnInPlace::TurnInPlace(SwerveDrive *swerve, DriveState state, units::degree_t goal) :
m_swerve(swerve),
m_swerveAlignUtil(swerve),
m_constraints(SwerveDriveConstants::kMaxAngularVelocity, SwerveDriveConstants::kMaxAngularAcceleration),
m_rotationPIDController(SwerveDriveConstants::kPRot, SwerveDriveConstants::kIRot, SwerveDriveConstants::kDRot, m_constraints),
m_withinThresholdLoops(0),
m_goal(0.0_deg) {
  (void)AutoConstants::kAutonomousPaths[0];
  (void)VisionConstants::kSyncBytes[0];
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve);
  m_rotationPIDController.EnableContinuousInput(0_deg, 360_deg);
  
  switch (state) {
    case(DriveState::HeadingControl) :
      break;
    case(DriveState::RotationVelocityControl) :
      break;
    case(DriveState::SpeakerAlign) :
      m_goal = m_swerveAlignUtil.GetSpeakerGoalAngle();
      break;
    case(DriveState::SpeakerAlignTranslationAlgorithm) :
      m_goal = m_swerveAlignUtil.GetSpeakerGoalAngleTranslation();
      break;
    case(DriveState::ArbitraryAngleAlign) :
      m_goal = goal;
      if (frc::DriverStation::GetAlliance()) {
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
          if (m_goal > 0.0_deg) {
            m_goal -= 180.0_deg;
          } else {
            m_goal += 180.0_deg;
          }
        }
      }
      break;
    case(DriveState::SourceAlign) :
      {
        auto allianceSide = frc::DriverStation::GetAlliance();
        if (allianceSide) {
          if (allianceSide.value() == frc::DriverStation::Alliance::kBlue) {
            m_goal = SwerveDriveConstants::kBlueSourceAlignTarget;
          } else {
            m_goal = SwerveDriveConstants::kRedSourceAlignTarget;
          }
        }
      }
      break;
    default :
      break;
  }
}

//initialize all objects needed including the PID Controller enabling and variables to determien needed drive state of robot

// Called when the command is initially scheduled.
void TurnInPlace::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TurnInPlace::Execute() {
  // Every loop, check if we're at ur goal (within 1 degree) and set to 0 if we aren't
  // After it reaches a certain amount of loops, end the command
  if (fabs(m_swerve->GetNormalizedYaw().value() - m_goal.value()) < AutoConstants::kAutoAlignTolerance)
    m_withinThresholdLoops++;
  else
    m_withinThresholdLoops = 0;

  auto rot = GetDesiredRotationalVelocity();
  m_swerve->Drive(0.0_mps, 0.0_mps, rot, true, frc::Translation2d{}, SwerveDriveConstants::kShouldDecelerate);
}

//If the yaw (angle) of swerve compared to the desired target is less than the tolerance, than the threshold will increase

// Called once the command ends or is interrupted.
void TurnInPlace::End(bool interrupted) {}

// Returns true when the command should end.
bool TurnInPlace::IsFinished() {
  return m_withinThresholdLoops >= 10;
}

// returns radians per second of the PID Controller calculating the desired velocity with the RotVel control
units::angular_velocity::radians_per_second_t TurnInPlace::GetDesiredRotationalVelocity() {
  // Return next velocity in radians per second as calculated by PIDController and limited by rotLimiter
  units::angular_velocity::radians_per_second_t rot = 
    units::angular_velocity::radians_per_second_t{m_rotationPIDController.Calculate(m_swerve->GetNormalizedYaw(), m_goal)
    * SwerveDriveConstants::kMaxAngularVelocity};

  return rot;
}