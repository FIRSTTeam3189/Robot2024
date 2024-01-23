// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunShooter
    : public frc2::CommandHelper<frc2::Command, RunShooter> {
 public:
  RunShooter(Shooter *m_shooter, double topPower, double bottomPower);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Shooter *m_shooter;
  double m_topPower;
  double m_bottomPower;
};