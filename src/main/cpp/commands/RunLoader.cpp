// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunLoader.h"

RunLoader::RunLoader(Shooter *shooter, double loadPower, double shootPower) : 
m_shooter(shooter),
m_loadPower(loadPower), 
m_shootPower(shootPower),
m_isFinished(false) {
  AddRequirements(shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RunLoader::Initialize() {
  m_shooter->SetLoaderPower(m_loadPower);
  m_shooter->SetRollerPower(m_shootPower);
}

// Called repeatedly when this Command is scheduled to run
void RunLoader::Execute() {
  if (m_shooter->NoteDetected())
    m_isFinished = true;
  else 
    m_isFinished = false;
}

// Called once the command ends or is interrupted.
void RunLoader::End(bool interrupted) {
  m_shooter->SetLoaderPower(0.0);
  m_shooter->SetRollerPower(0.0);
}

// Returns true when the command should end.
bool RunLoader::IsFinished() {
  return m_isFinished;
}
