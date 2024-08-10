// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Translation2d.h>
#include "subsystems/SwerveDrive.h"

class SwerveAlignUtil {
 public:
  SwerveAlignUtil(SwerveDrive *swerve);
  units::degree_t GetSpeakerGoalAngle();
  units::degree_t GetSpeakerGoalAngleTranslation();
  frc::Pose3d GetSpeakerPoseAllianceCompensated();

 private:
  SwerveDrive *m_swerve;
};
