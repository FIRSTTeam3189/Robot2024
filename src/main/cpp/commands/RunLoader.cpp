// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunLoader.h"

RunLoader::RunLoader(Shooter *shooter, double power) : m_loader(shooter), m_loaderPower(power), m_isFinished(false) {
  AddRequirements(shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RunLoader::Initialize() {
  m_loader->SetLoaderPower(m_loaderPower);
}

// Called repeatedly when this Command is scheduled to run
void RunLoader::Execute() {
  if (m_loader->NoteDetected())
    m_isFinished = true;
  else 
    m_isFinished = false;
}

// Called once the command ends or is interrupted.
void RunLoader::End(bool interrupted) {
  m_loader->SetLoaderPower(0.0);
}

// Returns true when the command should end.
bool RunLoader::IsFinished() {
  return m_isFinished;
}
