// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/ShooterAlignUtil.h"

ShooterAlignUtil::ShooterAlignUtil(SwerveDrive *swerve, Shooter *shooter) :
m_swerve(swerve),
m_shooter(shooter) {
  // Suppress unused warnings
  (void)AutoConstants::kAutonomousPaths[0];
  (void)VisionConstants::kSyncBytes[0];
}

units::meter_t ShooterAlignUtil::GetDistanceToSpeaker() {
    frc::Pose3d speakerPose = GetSpeakerPoseAllianceCompensated();

    auto currentPose = m_swerve->GetEstimatedPose();

    auto speakerTranslation = speakerPose.Translation().ToTranslation2d();
    auto robotTranslation = currentPose.Translation();

    units::meter_t distance = speakerTranslation.Distance(robotTranslation);

    frc::SmartDashboard::PutNumber("Speaker translation X: ", speakerTranslation.X().value());
    frc::SmartDashboard::PutNumber("Speaker translation Y: ", speakerTranslation.Y().value());

    frc::SmartDashboard::PutNumber("Robot translation X ", robotTranslation.X().value());
    frc::SmartDashboard::PutNumber("Robot translation Y ", robotTranslation.Y().value());

    frc::SmartDashboard::PutNumber("Robot Distance to Target: ", distance.value());

    return distance;
}

units::degree_t ShooterAlignUtil::GetShooterGoalInterpolating(units::meter_t distance) {
    // Use map to find nearest two distances to our actual distance
    // Could replace this linear search with more efficient if necessary
    // Start from highest distance, end when we reach a distance lower than our actual one
    // int lastElementIndex = ShooterConstants::kShooterKnownAngles.size() - 1;
    // for(int i = 0; i < ShooterConstants::kShooterKnownAngles.size(); i++) {
    //     units::meter_t knownDistance = ShooterConstants::kShooterKnownAngles;
    // }

    return 0.0_deg;
}

frc::Pose3d ShooterAlignUtil::GetSpeakerPoseAllianceCompensated() {
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