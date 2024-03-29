// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// runs the shooter to desired power (m_power) and then stops when it ends

#include "commands/RunShooter.h"

RunShooter::RunShooter(Shooter *shooter, double power) : 
m_shooter(shooter), m_power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void RunShooter::Initialize() {
  m_shooter->SetRollerPower(m_power);
}

// Called repeatedly when this Command is scheduled to run
void RunShooter::Execute() {}

// Called once the command ends or is interrupted.
void RunShooter::End(bool interrupted) {
  m_shooter->SetRollerPower(0.0);
  m_shooter->SetLoaderPower(0.0);
}

// Returns true when the command should end.
bool RunShooter::IsFinished() {
  return false;
}
