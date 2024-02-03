// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetIntakeRotation.h"

SetIntakeRotation::SetIntakeRotation(Intake *intake, double target) : m_intake(intake), m_target(target) {
  AddRequirements(intake);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetIntakeRotation::Initialize() {
  m_intake->SetRotation(m_target);
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeRotation::Execute() {

}

// Called once the command ends or is interrupted.
void SetIntakeRotation::End(bool interrupted) {
  m_intake->SetRotationPower(0.0);
}

// Returns true when the command should end.
bool SetIntakeRotation::IsFinished() {
  if (abs(m_target - m_intake->GetRotation()) < IntakeConstants::kRotationStopDistance) {
    return true;
  }
  return false;
}
