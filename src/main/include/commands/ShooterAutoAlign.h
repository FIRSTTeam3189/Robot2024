// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/DriverStation.h>
#include "subsystems/Shooter.h"
#include "subsystems/PoseEstimatorHelper.h"
#include "subsystems/Vision.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShooterAutoAlign
    : public frc2::CommandHelper<frc2::Command, ShooterAutoAlign> {
 public:
  ShooterAutoAlign(Shooter *shooter, PoseEstimatorHelper *estimator, Vision *vision, bool shouldFinish);
  units::degree_t CalculateShooterAngle();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Shooter *m_shooter;
  PoseEstimatorHelper *m_helper;
  Vision *m_vision;
  bool m_shouldFinish;
};
