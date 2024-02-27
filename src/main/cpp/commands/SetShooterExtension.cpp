// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetShooterExtension.h"

SetShooterExtension::SetShooterExtension(Shooter *shooter, double target) : m_shooter(shooter), m_target(target) {
  AddRequirements(shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetShooterExtension::Initialize() {
  m_shooter->SetExtension(m_target);
}

// Called repeatedly when this Command is scheduled to run
void SetShooterExtension::Execute() {}

// Called once the command ends or is interrupted.
void SetShooterExtension::End(bool interrupted) {
  m_shooter->SetExtensionPower(0.0);
}

// Returns true when the command should end.
bool SetShooterExtension::IsFinished() {
  if (abs(m_target - m_shooter->GetExtension()) < ShooterConstants::kExtensionStopDistance) {
    return true;
  }
  return false;
}
