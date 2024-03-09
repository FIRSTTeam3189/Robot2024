// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpIntake.h"

AmpIntake::AmpIntake(Intake *intake, double speed) : 
m_intake(intake), m_speed(speed), m_shouldFinish(false) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void AmpIntake::Initialize() {
  m_intake->SetRollerPower(m_speed);
}

// Called repeatedly when this Command is scheduled to run
void AmpIntake::Execute() {
  m_shouldFinish = false;
}

// Called once the command ends or is interrupted.
void AmpIntake::End(bool interrupted) {
  m_intake->SetRotation(IntakeConstants::kRetractTarget);
  m_intake->SetRollerPower(0.0);
}

// Returns true when the command should end.
bool AmpIntake::IsFinished() {
  return m_shouldFinish;
}
