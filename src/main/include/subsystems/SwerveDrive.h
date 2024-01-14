// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Translation2d.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "util/SwerveModule.h"

struct SwerveModules {
  frc::Translation2d m_frontLeftLocation;
  frc::Translation2d m_frontRightLocation;
  frc::Translation2d m_backLeftLocation;
  frc::Translation2d m_backRightLocation;

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;
};

class SwerveDrive : public frc2::SubsystemBase {
 public:
  SwerveDrive();
  void SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates);
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed,
             units::radians_per_second_t rot,
             bool fieldRelative,
             frc::Translation2d centerOfRotation);
  void Lock();
  void Stop();
  double GetNormalizedYaw();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveModules m_modules;
  ctre::phoenix6::hardware::Pigeon2 m_pigeon;

  // Tuning mode preference -- when true, will constantly update module preferences
  std::string_view m_tuningModeKey = "Tuning Mode?";
};