// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// sets intake to power

#include "commands/RunIntake.h"

RunIntake::RunIntake(Intake *intake, double rollerPower) :
m_intake(intake), m_rollerPower(rollerPower) {
  AddRequirements(intake);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RunIntake::Initialize() {
  m_intake->SetRollerPower(m_rollerPower);
}

// Called repeatedly when this Command is scheduled to run
void RunIntake::Execute() {}

// Called once the command ends or is interrupted.
void RunIntake::End(bool interrupted) {
  m_intake->SetRollerPower(0.0);
}

// Returns true when the command should end.
bool RunIntake::IsFinished() {
  return false;
}