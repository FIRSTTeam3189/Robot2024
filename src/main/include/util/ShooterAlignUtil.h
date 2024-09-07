// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose3d.h>
#include "subsystems/PoseEstimatorHelper.h"
#include "Constants/VisionConstants.h"
#include "Constants/ShooterConstants.h"

class ShooterAlignUtil {
 public:
  ShooterAlignUtil(PoseEstimatorHelper *estimator);
  units::meter_t GetDistanceToSpeaker();
  units::degree_t GetShooterGoalInterpolating(units::meter_t distance);
  frc::Pose3d GetSpeakerPoseAllianceCompensated();

 private:
  // Shooter only uses the pose from the Swerve so we can just simply give it the PoseEstimatorHelper
  PoseEstimatorHelper *m_estimator;
};
