// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TransferLoader.h"

TransferLoader::TransferLoader(Shooter *shooter, double loadPower, double shootPower) :
m_shooter(shooter),
m_loadPower(loadPower), 
m_shootPower(shootPower),
m_isFinished(false) {
  // Use addRequirements() here to declare subsystem dependencies.
  (void)AutoConstants::kAutonomousPaths[0];
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void TransferLoader::Initialize() {
   m_shooter->SetLoaderPower(m_loadPower);
   m_shooter->SetRollerPower(m_shootPower);
}

// Called repeatedly when this Command is scheduled to run
void TransferLoader::Execute() {
   if (m_shooter->NoteDetected())
    m_isFinished = false;
  else 
    m_isFinished = true;
}

// Called once the command ends or is interrupted.
void TransferLoader::End(bool interrupted) {
   m_shooter->SetLoaderPower(0.0);
   m_shooter->SetRollerPower(0.0);
   
}

// Returns true when the command should end.
bool TransferLoader::IsFinished() {
  return m_isFinished;
}
