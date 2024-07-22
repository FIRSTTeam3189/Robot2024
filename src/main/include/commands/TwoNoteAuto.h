// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include "subsystems/Intake.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/Shooter.h"
#include "commands/SetIntakeRotation.h"
#include "commands/RunIntake.h"
#include "commands/RunShooter.h"
#include "commands/RunLoader.h"
#include "commands/SetIntakeRotation.h"
#include "commands/SetShooterRotation.h"
#include "Constants/AutoConstants.h"
#include "Constants/SwerveDriveConstants.h"

class TwoNoteAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 TwoNoteAuto> {
 public:
  TwoNoteAuto(SwerveDrive *swerve, Intake *intake, Shooter *shooter, StartingPosition position);

 private:
  SwerveDrive *m_swerve;
  Intake *m_intake;
  Shooter *m_shooter;
  StartingPosition m_position;
  frc::PIDController m_rotationPIDController;
};
