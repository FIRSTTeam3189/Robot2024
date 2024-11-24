// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/DriverStation.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/PS5Controller.h>
#include <frc2/command/button/CommandJoystick.h>
#include "subsystems/SwerveDrive.h"
#include "Constants/OperatorConstants.h"
#include "util/SwerveAlignUtil.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

class Drive
    : public frc2::CommandHelper<frc2::Command, Drive> {
 public:
  Drive(frc::PS5Controller *joystick, SwerveDrive *swerveDrive, DriveState driveState, units::degree_t arbitraryAngle = 0.0_deg);
  units::angular_velocity::radians_per_second_t GetDesiredRotationalVelocity();
  units::angular_velocity::radians_per_second_t GetRotVelSpeakerAlign();
  units::angular_velocity::radians_per_second_t GetRotVelSpeakerAlignTranslation();
  void UpdatePreferences();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  frc::PS5Controller *m_bill;
  SwerveDrive *m_swerveDrive;
  // Needs to be here rather than the subsystem because the PID controller is utilized in the Drive command
  SwerveAlignUtil m_swerveAlignUtil;
  frc::PIDController m_rotationPIDController;
  DriveState m_driveState;
  units::degree_t m_arbitraryAngle;
  frc::SlewRateLimiter<units::scalar> m_xSpeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_ySpeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};
  double m_goalAngle;
  double m_lastAngle;
  std::optional<frc::DriverStation::Alliance> m_allianceSide;
  std::string m_rotationPKey;
  std::string m_rotationIKey;
  std::string m_rotationDKey;
};
