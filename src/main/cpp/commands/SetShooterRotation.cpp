// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetShooterRotation.h"

SetShooterRotation::SetShooterRotation(Shooter *shooter, double position) : m_shooter(shooter), m_position(position){
  AddRequirements(shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetShooterRotation::Initialize() {
  m_shooter->SetRotation(m_position);
}

// Called repeatedly when this Command is scheduled to run
void SetShooterRotation::Execute() {}

// Called once the command ends or is interrupted.
void SetShooterRotation::End(bool interrupted) {}

// Returns true when the command should end.
bool SetShooterRotation::IsFinished() {
  return false;
}
