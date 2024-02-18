// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpScore.h"

AmpScore::AmpScore(Intake *intake, units::degree_t target) : m_intake(intake), m_target(target) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void AmpScore::Initialize() {
  m_intake->SetRotation(m_target);
}

// Called repeatedly when this Command is scheduled to run
void AmpScore::Execute() {
  if (abs(m_target.value() - m_intake->GetRotation().value()) < IntakeConstants::kRotationStopDistance.value() 
    && m_target != IntakeConstants::kRetractTarget) {
    m_intake->SetRollerPower(IntakeConstants::kAmpScorePower);
  }
}

// Called once the command ends or is interrupted.
void AmpScore::End(bool interrupted) {}

// Returns true when the command should end.
bool AmpScore::IsFinished() {
  return false;
}
