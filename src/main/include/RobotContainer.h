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
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/AmpIntake.h"
#include "commands/Drive.h"
#include "commands/MobilityAuto.h"
#include "commands/RunClimber.h"
#include "commands/RunIntake.h"
#include "commands/RunLoader.h"
#include "commands/RunShooter.h"
#include "commands/SetIntakeRotation.h"
#include "commands/SetShooterRotation.h"
#include "commands/ShooterAutoAlign.h"
#include "commands/TurnInPlace.h"
#include "commands/TwoNoteAuto.h"
#include "subsystems/LED.h"
#include "subsystems/MusicSystem.h"
#include "subsystems/Climber.h"
#include "subsystems/Vision.h"
#include "subsystems/PoseEstimatorHelper.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

enum class SuperstructureState { Default, Climb };

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

  frc2::Command* GetAutonomousCommand();
  SuperstructureState GetSuperstructureState();
  bool IsClimbState();
  BrakeMode GetBrakeMode();
  void SetAllCoast();
  void SetAllNormalBrakeMode();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandJoystick m_bill{OperatorConstants::kDriverControllerPort};
  frc2::CommandJoystick m_ted{OperatorConstants::kCoDriverControllerPort};
  frc2::CommandJoystick m_test{OperatorConstants::kTestControllerPort};

  // The robot's subsystems are defined here...
  Climber *m_climber = new Climber();
  PoseEstimatorHelper *m_poseEstimator = new PoseEstimatorHelper();
  Vision *m_vision = new Vision(m_poseEstimator);
  Shooter *m_shooter = new Shooter(m_poseEstimator);
  SwerveDrive *m_swerveDrive = new SwerveDrive(m_poseEstimator);
  Intake *m_intake = new Intake();
  MusicSystem *m_musicSystem = new MusicSystem(m_swerveDrive->GetMotorsForMusic());

  frc::SendableChooser<frc2::Command*> m_chooser;

  SuperstructureState m_superstructureState = SuperstructureState::Default;
  DriveState m_driveState = DriveState::HeadingControl;
  units::degree_t m_driveAligntarget;

  void ConfigureDriverBindings();
  void ConfigureCoDriverBindings();
  void ConfigureTestBindings();
  void CreateAutoPaths();
  void RegisterAutoCommands();
};