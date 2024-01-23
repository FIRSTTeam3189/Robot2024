// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/pch.h"
#include "commands/SetIntakeExtension.h"

SetIntakeExtension::SetIntakeExtension(Intake *intake, double position) : m_intake(intake), m_position(position) {
  AddRequirements(intake);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetIntakeExtension::Initialize() {
  m_intake->SetExtension(m_position);
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeExtension::Execute() {

}

// Called once the command ends or is interrupted.
void SetIntakeExtension::End(bool interrupted) {

}

// Returns true when the command should end.
bool SetIntakeExtension::IsFinished() {
  return false;
}
