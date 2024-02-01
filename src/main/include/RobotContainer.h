// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>

#include "Constants.h"
#include "commands/JoystickDrive.h"
#include "commands/RunShooter.h"
#include "commands/SetIntakeRotation.h"
#include "commands/SetShooterRotation.h"
#include "commands/RunIntake.h"
#include "subsystems/LED.h"
#include "subsystems/MusicSystem.h"
#include "subsystems/Climber.h"
#include "subsystems/Vision.h"
#include "subsystems/PoseEstimatorHelper.h"

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
  Climber *m_climber = new Climber();
  PoseEstimatorHelper *m_helper = new PoseEstimatorHelper();
  Vision *m_vision = new Vision(m_helper);
  Shooter *m_shooter = new Shooter();
  SwerveDrive *m_swerveDrive = new SwerveDrive(m_helper);
  Intake *m_intake = new Intake();
  MusicSystem *m_musicSystem = new MusicSystem(m_swerveDrive->GetMotorsForMusic());

  bool m_isSpecialHeadingMode = true;
  bool m_isFieldRelative = true;

  void ConfigureDriverBindings();
  void ConfigureCoDriverBindings();
};