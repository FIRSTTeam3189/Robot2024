// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/DriverStation.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/CommandJoystick.h>
#include "subsystems/SwerveDrive.h"
#include "Constants/OperatorConstants.h"

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
  Drive(frc2::CommandJoystick *joystick, SwerveDrive *swerveDrive, 
                bool isSpecialHeadingMode, bool isFieldRelative = true, bool shouldAlignSpeaker = false);
  units::angular_velocity::radians_per_second_t GetDesiredRotationalVelocity();
  units::angular_velocity::radians_per_second_t GetRotVelSpeakerAlign();
  void UpdatePreferences();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  frc2::CommandJoystick *m_bill;
  SwerveDrive *m_swerveDrive;
  frc::PIDController m_rotationPIDController;
  bool m_isSpecialHeadingMode;
  bool m_isFieldRelative;
  bool m_shouldAlignSpeaker;
  frc::SlewRateLimiter<units::scalar> m_xSpeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_ySpeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
  double m_goalAngle;
  double m_lastAngle;
  std::optional<frc::DriverStation::Alliance> m_allianceSide;
  std::string m_rotationPKey;
  std::string m_rotationIKey;
  std::string m_rotationDKey;
};
