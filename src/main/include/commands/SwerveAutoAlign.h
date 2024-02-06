// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/DriverStation.h>
#include <frc/controller/ProfiledPIDController.h>
#include "subsystems/SwerveDrive.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SwerveAutoAlign
    : public frc2::CommandHelper<frc2::Command, SwerveAutoAlign> {
 public:
  SwerveAutoAlign(SwerveDrive *swerve, bool shouldAlignSpeaker, units::degree_t goal = 0.0_deg);
  units::angular_velocity::radians_per_second_t GetDesiredRotationalVelocity();
  units::degree_t GetSpeakerGoalAngle();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDrive *m_swerve;
  frc::TrapezoidProfile<units::degrees>::Constraints m_constraints;
  frc::ProfiledPIDController<units::degrees> m_rotationPIDController;
  int m_withinThresholdLoops;
  units::degree_t m_goal;
};
