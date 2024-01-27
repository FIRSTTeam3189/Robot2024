// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetIntakeRotation.h"

SetIntakeRotation::SetIntakeRotation(Intake *intake, double position) : m_intake(intake), m_position(position) {
  AddRequirements(intake);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetIntakeRotation::Initialize() {
  m_intake->SetRotation(m_position);
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeRotation::Execute() {

}

// Called once the command ends or is interrupted.
void SetIntakeRotation::End(bool interrupted) {

}

// Returns true when the command should end.
bool SetIntakeRotation::IsFinished() {
  return false;
}
