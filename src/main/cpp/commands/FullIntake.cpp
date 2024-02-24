// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FullIntake.h"

FullIntake::FullIntake(Intake *intake, double speed, units::degree_t target) : 
m_intake(intake), m_speed(speed), m_target(target), m_shouldFinish(false) {
  // Use addRequirements() here to declare subsystem dependencies.
  (void)AutoConstants::kAutonomousPaths[0];
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void FullIntake::Initialize() {
  m_intake->SetRotation(m_target);
  m_intake->SetRollerPower(m_speed);
}

// Called repeatedly when this Command is scheduled to run
void FullIntake::Execute() {
  if (m_intake->NoteDetected()) 
    m_shouldFinish = true;
  else
    m_shouldFinish = false;
}

// Called once the command ends or is interrupted.
void FullIntake::End(bool interrupted) {
  m_intake->SetRotation(IntakeConstants::kRetractTarget);
  m_intake->SetRollerPower(0.0);
}

// Returns true when the command should end.
bool FullIntake::IsFinished() {
  return m_shouldFinish;
}
