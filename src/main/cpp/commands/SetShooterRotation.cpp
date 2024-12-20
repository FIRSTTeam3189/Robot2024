// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Runs shooter rotation to desired value

#include "commands/SetShooterRotation.h"

SetShooterRotation::SetShooterRotation(Shooter *shooter, ShooterState state) : m_shooter(shooter), m_state(state){
  AddRequirements(shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetShooterRotation::Initialize() {
  m_shooter->SetState(m_state);
  m_shooter->SetActive(true);
  if(m_state == ShooterState::Close || m_state == ShooterState::AutoScore) {
    m_shooter->SetRollerPower(ShooterConstants::kShootPower);
  }
}

// Called repeatedly when this Command is scheduled to run
void SetShooterRotation::Execute() {}

// Called once the command ends or is interrupted.
void SetShooterRotation::End(bool interrupted) {
  m_shooter->SetActive(false);
}

// Returns true when the command should end.
bool SetShooterRotation::IsFinished() {
  if (abs(m_shooter->GetTarget().value() - m_shooter->GetRotation().value()) < ShooterConstants::kRotationStopDistance.value()) {
    return true;
  }
  return false;
}

// based on desired and actual rotation, it returns true to end when reaches constant value