// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DriverStation.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Translation2d.h>
#include "subsystems/Shooter.h"
#include "subsystems/SwerveDrive.h"

class ShooterAlignUtil {
 public:
  ShooterAlignUtil(SwerveDrive *swerve, Shooter *shooter);
  units::meter_t GetDistanceToSpeaker();
  units::degree_t GetShooterGoalInterpolating(units::meter_t distance);
  frc::Pose3d GetSpeakerPoseAllianceCompensated();

 private:
  SwerveDrive *m_swerve;
  Shooter *m_shooter;
};
