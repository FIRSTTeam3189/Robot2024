// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/ShooterAlignUtil.h"

ShooterAlignUtil::ShooterAlignUtil(PoseEstimatorHelper *estimator) :
m_poseEstimator(estimator)
{
}

units::meter_t ShooterAlignUtil::GetDistanceToSpeaker() {
    frc::Pose3d speakerPose = GetSpeakerPoseAllianceCompensated();

    auto currentPose = m_poseEstimator->GetEstimatedPose();

    auto speakerTranslation = speakerPose.Translation().ToTranslation2d();
    auto robotTranslation = currentPose.Translation();

    units::meter_t distance = speakerTranslation.Distance(robotTranslation);

    frc::SmartDashboard::PutNumber("Speaker translation X", speakerTranslation.X().value());
    frc::SmartDashboard::PutNumber("Speaker translation Y", speakerTranslation.Y().value());

    frc::SmartDashboard::PutNumber("Robot translation X", robotTranslation.X().value());
    frc::SmartDashboard::PutNumber("Robot translation Y", robotTranslation.Y().value());

    frc::SmartDashboard::PutNumber("Robot Distance to Target", distance.value());

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

    for (size_t i = 1; i < ShooterConstants::kShooterKnownDistances.size(); i++) {
      // Only run algorithm if real distance is in the range of the distances in the dataset`
      if (ShooterConstants::kShooterKnownDistances.at(i) > distance){
        units::degree_t upperAngle = ShooterConstants::kShooterKnownAngles.at(i);
        units::degree_t lowerAngle = ShooterConstants::kShooterKnownAngles.at(i - 1);

        units::meter_t upperDistance = ShooterConstants::kShooterKnownDistances.at(i);
        units::meter_t lowerDistance = ShooterConstants::kShooterKnownDistances.at(i-1);

        units::meter_t difference = distance - lowerDistance;
        double slope = (upperAngle.value() - lowerAngle.value()) / (upperDistance.value() - lowerDistance.value());
        
        // Lower angle is the base, we interpolate along the line using the given slope
        // So that the difference between the lower distance and the input distance
        // is multiplied by the slope of the line
        units::degree_t goalAngle = lowerAngle + units::degree_t{slope * difference.value()};
        frc::SmartDashboard::PutNumber("Shooter interpolated goal angle", goalAngle.value());
        frc::SmartDashboard::PutNumber("Lower angle", lowerAngle.value());
        frc::SmartDashboard::PutNumber("Upper angle", upperAngle.value());
        frc::SmartDashboard::PutNumber("Lower distance", lowerDistance.value());
        frc::SmartDashboard::PutNumber("Upper distance", upperDistance.value());
        frc::SmartDashboard::PutNumber("Slope", slope);
        frc::SmartDashboard::PutNumber("Difference", difference.value());

        return lowerAngle + units::degree_t{slope * difference.value()};
      }
    }

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