// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/pch.h"
#include "commands/SetShooterExtension.h"

SetShooterExtension::SetShooterExtension(Shooter *shooter, double position) : m_shooter(shooter), m_position(position){
  AddRequirements(shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetShooterExtension::Initialize() {
  m_shooter->SetExtension(m_position);
}

// Called repeatedly when this Command is scheduled to run
void SetShooterExtension::Execute() {}

// Called once the command ends or is interrupted.
void SetShooterExtension::End(bool interrupted) {}

// Returns true when the command should end.
bool SetShooterExtension::IsFinished() {
  return false;
}
