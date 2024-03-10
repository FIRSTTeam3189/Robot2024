// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Sets the intake rotation to desired state

#include "commands/SetIntakeRotation.h"

SetIntakeRotation::SetIntakeRotation(Intake *intake, IntakeState state) : m_intake(intake), m_state(state) {
  AddRequirements(intake);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetIntakeRotation::Initialize() {
  m_intake->SetState(m_state);
  m_intake->SetActive(true);
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeRotation::Execute() {}

// Called once the command ends or is interrupted.
void SetIntakeRotation::End(bool interrupted) {
  m_intake->SetActive(false);
  // m_intake->SetRotationPower(0.0);
}

// Returns true when the command should end.
bool SetIntakeRotation::IsFinished() {
  if (abs(m_intake->GetTarget().value() - m_intake->GetRotation().value()) < IntakeConstants::kRotationStopDistance.value()) {
    return true;
  }
  return false;
}

// based on target desired and actual rotation, if the difference is less than the constant stop distance value, then the intake is done 
