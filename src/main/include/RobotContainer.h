// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include "commands/JoystickDrive.h"
#include "commands/RunIntake.h"
#include "commands/RunShooter.h"
#include "commands/SetIntakeExtension.h"
#include "commands/SetShooterExtension.h"
#include "subsystems/LED.h"
#include "subsystems/Orchestra.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandJoystick m_bill{OperatorConstants::kDriverControllerPort};
  frc2::CommandJoystick m_ted{OperatorConstants::kCoDriverControllerPort};

  // The robot's subsystems are defined here...
  Shooter *m_shooter = new Shooter();
  SwerveDrive *m_swerveDrive = new SwerveDrive();
  Intake *m_intake = new Intake();
  Orchestra *m_orchestra = new Orchestra(m_swerveDrive->GetMotorsForMusic());

  bool m_isSpecialHeadingMode = true;
  bool m_isFieldRelative = true;

  void ConfigureBindings();
};