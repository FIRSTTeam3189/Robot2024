// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Shooter.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunLoader
    : public frc2::CommandHelper<frc2::Command, RunLoader> {
 public:
  RunLoader(Shooter *shooter, double loadPower, double shootPower, ShooterEndCondition endCondition = ShooterEndCondition::None);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
   Shooter *m_shooter;
   double m_loadPower;
   double m_shootPower;
   bool m_isFinished;
   ShooterEndCondition m_endCondition;
};
