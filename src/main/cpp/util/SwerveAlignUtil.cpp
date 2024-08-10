// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/SwerveAlignUtil.h"

SwerveAlignUtil::SwerveAlignUtil(SwerveDrive *swerve) :
m_swerve(swerve) {
  // Suppress unused warnings
  (void)AutoConstants::kAutonomousPaths[0];
  (void)VisionConstants::kSyncBytes[0];
}

frc::Pose3d SwerveAlignUtil::GetSpeakerPoseAllianceCompensated() {
  frc::Pose3d tagPose = VisionConstants::kTagPoses.at(6);
  auto allianceSide = frc::DriverStation::GetAlliance();
  if (allianceSide) {
    if (allianceSide.value() == frc::DriverStation::Alliance::kRed) {
      tagPose = VisionConstants::kTagPoses.at(3);
    } else {
      tagPose = VisionConstants::kTagPoses.at(6);
    }
  }

  return tagPose;
}

units::degree_t SwerveAlignUtil::GetSpeakerGoalAngle() {
  auto allianceSide = frc::DriverStation::GetAlliance();

  // which speaker it should align to based on alliance found by tag
  frc::Pose3d tagPose = GetSpeakerPoseAllianceCompensated();

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

units::degree_t SwerveAlignUtil::GetSpeakerGoalAngleTranslation() {
  frc::Pose3d tagPose = GetSpeakerPoseAllianceCompensated();

  auto currentPose = m_swerve->GetEstimatedPose();
  
  auto speakerTranslation = tagPose.Translation().ToTranslation2d();
  auto robotTranslation = currentPose.Translation();
  // Probably can just subtract but not sure
  frc::Translation2d robotToTarget = speakerTranslation + (-robotTranslation);

  units::degree_t goalAngle = robotToTarget.Angle().Degrees();
  // Might need to flip angle based on alliance side, but maybe not (see atan vs translation2d direction)
  // if (allianceSide) {
  //   if (allianceSide.value() == frc::DriverStation::Alliance::kRed) {
  //       goalAngle += 180.0_deg;
  //       if (goalAngle > 180.0_deg)
  //         goalAngle -= 360.0_deg;
  //   }
  // }

  frc::SmartDashboard::PutNumber("Swerve auto align angle translation", goalAngle.value());
  return goalAngle;
}