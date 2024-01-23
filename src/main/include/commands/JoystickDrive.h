// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class JoystickDrive
    : public frc2::CommandHelper<frc2::Command, JoystickDrive> {
 public:
  JoystickDrive(frc2::CommandJoystick *joystick, SwerveDrive *swerveDrive, bool isSpecialHeadingMode, bool isFieldRelative = true);
  units::angular_velocity::radians_per_second_t GetDesiredRotationalVelocity();

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
  frc::SlewRateLimiter<units::scalar> m_xSpeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_ySpeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
};
