// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// sets climber to desoredd power based on it climbing or not

#include "commands/RunClimber.h"

RunClimber::RunClimber(Climber *climber, double power) : m_climber(climber),
                                                         m_power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(climber);
}

// Called when the command is initially scheduled.
void RunClimber::Initialize() {
  m_climber->SetPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void RunClimber::Execute() {}

// Called once the command ends or is interrupted.
void RunClimber::End(bool interrupted) {
  m_climber->SetPower(0.0);
}

// Returns true when the command should end.
bool RunClimber::IsFinished() {
  return false;
}
