// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/pch.h"
#include "commands/RunShooter.h"

RunShooter::RunShooter(Shooter *shooter, double topPower, double bottomPower) : 
m_shooter(shooter), m_topPower(topPower), m_bottomPower(bottomPower) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void RunShooter::Initialize() {
  m_shooter->SetTopPower(m_topPower);
  m_shooter->SetBottomPower(m_bottomPower);
}

// Called repeatedly when this Command is scheduled to run
void RunShooter::Execute() {}

// Called once the command ends or is interrupted.
void RunShooter::End(bool interrupted) {
  m_shooter->SetTopPower(0.0);
  m_shooter->SetBottomPower(0.0);
}

// Returns true when the command should end.
bool RunShooter::IsFinished() {
  return false;
}
