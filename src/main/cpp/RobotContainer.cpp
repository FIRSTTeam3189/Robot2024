// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  (void)VisionConstants::kSyncBytes[0];
  RegisterAutoCommands();
  
  // Initialize all of your commands and subsystems here 

  m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
  
  // m_intake->SetDefaultCommand(frc2::RunCommand([this] {
  //   m_intake->SetRotationPower(m_ted.GetRawAxis(OperatorConstants::kAxisLeftStickY));
  // },{m_intake}).ToPtr());

  // m_shooter->SetDefaultCommand(frc2::RunCommand([this]{
  //   auto input = m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY);
  //   if (fabs(input) > 0.05)
  //     m_shooter->SetRotationPower(-m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY));
  //   else
  //     m_shooter->SetRotationPower(0.02);
  //   // May need to change joystick axis
  // },{m_shooter}));
  
  // Configure the button bindings
  ConfigureDriverBindings();
  ConfigureCoDriverBindings();
  ConfigureSysIDBindings();
  CreateAutoPaths();
}

void RobotContainer::ConfigureDriverBindings() {
  // Bill controls
  frc2::Trigger toggleATan2RotButton{m_bill.Button(OperatorConstants::kButtonIDRightStick)};
  toggleATan2RotButton.OnTrue(
    frc2::InstantCommand([this]{
      if (m_driveState == DriveState::HeadingControl) {
        m_driveState = DriveState::RotationVelocityControl;
      } else if (m_driveState == DriveState::RotationVelocityControl) {
        m_driveState = DriveState::HeadingControl;
      }
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
    },{m_swerveDrive}).ToPtr()
  );

  frc2::Trigger fullIntakeButton{m_bill.Button(OperatorConstants::kButtonIDLeftBumper)};
  fullIntakeButton.OnTrue(frc2::SequentialCommandGroup(
    frc2::ParallelCommandGroup(
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetIntakeRotation(m_intake, IntakeState::Extended)
      ),
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetShooterRotation(m_shooter, ShooterState::Load)
      )
    ),
    frc2::ParallelCommandGroup(
      // RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0, ShooterEndCondition::EndOnSecondDetection),
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0, ShooterEndCondition::None),
      RunIntake(m_intake, IntakeConstants::kIntakePower)
    )
  ).ToPtr());
  fullIntakeButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_intake, m_shooter}),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Retracted),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    )
  ).ToPtr());

  frc2::Trigger ampIntakeButton{m_bill.Button(OperatorConstants::kButtonIDRightBumper)};
  ampIntakeButton.OnTrue(frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeState::Extended),
    AmpIntake(m_intake, IntakeConstants::kIntakePower)
  ).ToPtr());
  ampIntakeButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
    },{m_intake}),
    SetIntakeRotation(m_intake, IntakeState::Retracted)
  ).ToPtr());

  frc2::Trigger alignSpeakerButton{m_bill.Button(OperatorConstants::kButtonIDLeftTrigger)};
  alignSpeakerButton.OnTrue(frc2::InstantCommand([this]{
      m_driveState = DriveState::SpeakerAlign;
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
    },{m_swerveDrive}).ToPtr()
  );

  frc2::Trigger driveUnderStageButton{m_bill.Button(OperatorConstants::kButtonIDLeftTrigger)};
  driveUnderStageButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Far).ToPtr());
  driveUnderStageButton.OnFalse(SetShooterRotation(m_shooter, ShooterState::Retracted).ToPtr());

  frc2::Trigger ampScoreIntakeButton{m_bill.Button(OperatorConstants::kButtonIDRightTrigger)};
  ampScoreIntakeButton.OnTrue(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_driveState = DriveState::ArbitraryAngleAlign;
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState, SwerveDriveConstants::kAmpAlignTarget));
    },{m_swerveDrive}),
    SetIntakeRotation(m_intake, IntakeState::Amp)
  ).ToPtr());
  ampScoreIntakeButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(IntakeConstants::kAmpShootTime), 
      RunIntake(m_intake, IntakeConstants::kAmpScorePower)
    ),
    SetIntakeRotation(m_intake, IntakeState::Retracted),
    frc2::InstantCommand([this]{
      m_driveState = DriveState::HeadingControl;
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
    },{m_swerveDrive})
  ).ToPtr());

  frc2::Trigger resetPoseButton{m_bill.Button(OperatorConstants::kButtonIDTouchpad)};
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    if (frc::DriverStation::GetAlliance()) {
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}}, true);
      else
        m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{180.0_deg}}, true);
    }
  },{m_swerveDrive}).ToPtr());

  frc2::Trigger resetSpeakerPoseButton{m_bill.Button(OperatorConstants::kButtonIDCreate)};
  resetSpeakerPoseButton.OnTrue(frc2::InstantCommand([this]{
    if (frc::DriverStation::GetAlliance()) {
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        m_swerveDrive->SetPose(frc::Pose2d{0.92_m, 5.50_m, frc::Rotation2d{0.0_deg}}, false);
      else
        m_swerveDrive->SetPose(frc::Pose2d{15.579_m, 5.50_m, frc::Rotation2d{180.0_deg}}, false);
    }
  },{m_swerveDrive}).ToPtr());

  frc2::Trigger toggleClimbModeButton{m_bill.Button(OperatorConstants::kButtonIDMenu)};
  toggleClimbModeButton.OnTrue(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      if (GetSuperstructureState() == SuperstructureState::Default) {
        m_superstructureState = SuperstructureState::Climb;
      } else {
        m_superstructureState = SuperstructureState::Default;
      }
    }),
    frc2::ParallelCommandGroup(
      SetIntakeRotation(m_intake, IntakeState::Extended),
      SetShooterRotation(m_shooter, ShooterState::Retracted)
    )
  ).ToPtr());

  frc2::Trigger startingConfigButton{m_bill.Button(OperatorConstants::kButtonIDMicrophone)};
  startingConfigButton.OnTrue(frc2::ParallelCommandGroup(
    SetIntakeRotation(m_intake, IntakeState::Retracted),
    SetShooterRotation(m_shooter, ShooterState::StartingConfig)
  ).ToPtr());
  
  // frc2::Trigger extendIntakeButton{m_bill.Button(OperatorConstants::kButtonIDTriangle)};
  // extendIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeState::Extended).ToPtr());

  // frc2::Trigger retractIntakeButton{m_bill.Button(OperatorConstants::kButtonIDX)};
  // retractIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeState::Retracted).ToPtr());

  // frc2::Trigger runIntake{m_bill.Button(OperatorConstants::kButtonIDCircle)};
  // runIntake.OnTrue(frc2::InstantCommand([this]{
  //     m_intake->SetRollerPower(1.0);
  // },{m_intake}).ToPtr());
  // runIntake.OnFalse(frc2::InstantCommand([this]{
  //     m_intake->SetRollerPower(0.0);
  // },{m_intake}).ToPtr());

  // frc2::Trigger toggleSlowModeButton{m_bill.Button(OperatorConstants::kButtonIDRightBumper)};
  // toggleSlowModeButton.OnTrue(frc2::InstantCommand([this]{ m_swerveDrive->ToggleSlowMode(); },{m_swerveDrive}).ToPtr());
}

void RobotContainer::ConfigureCoDriverBindings() {
  // Ted controls
  frc2::Trigger directShooterLoadButton{m_ted.Button(OperatorConstants::kButtonIDLeftBumper)};
  directShooterLoadButton.OnTrue(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_driveState = DriveState::ArbitraryAngleAlign;
      auto target = 0.0_deg;
      if (frc::DriverStation::GetAlliance()) {
        if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
          target = SwerveDriveConstants::kBlueSourceAlignTarget;
        else
          target = SwerveDriveConstants::kRedSourceAlignTarget;
      }
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState, target));
    },{m_swerveDrive}),
    SetShooterRotation(m_shooter, ShooterState::DirectLoad),
    // RunLoader(m_shooter, ShooterConstants::kDirectLoadPower, ShooterConstants::kDirectLoadPower, ShooterEndCondition::EndOnFirstDetection),
    RunLoader(m_shooter, ShooterConstants::kDirectLoadPower, ShooterConstants::kDirectLoadPower, ShooterEndCondition::None),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Retracted),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    )
  ).ToPtr());
  directShooterLoadButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Retracted),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    ),
    frc2::InstantCommand([this]{
      m_driveState = DriveState::HeadingControl;
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
    },{m_swerveDrive})
  ).ToPtr());

  frc2::Trigger unloadButton{m_ted.Button(OperatorConstants::kButtonIDRightBumper)};
  unloadButton.OnTrue(RunLoader(m_shooter, ShooterConstants::kUnloadPower, 0.0).ToPtr());
  unloadButton.OnFalse(
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}).ToPtr()
  );

  // frc2::Trigger shooterAlignButton{m_ted.Button(OperatorConstants::kButtonIDLeftTrigger)};
  // shooterAlignButton.OnTrue(ShooterAutoAlign(m_shooter, m_estimator, m_vision).ToPtr());
  // shooterAlignButton.OnFalse(frc2::InstantCommand([this]{
  //   m_shooter->SetRotationPower(0.0);
  // },{m_shooter}).ToPtr());

  frc2::Trigger shootLowButton{m_ted.Button(OperatorConstants::kButtonIDLeftTrigger)};
  shootLowButton.OnTrue(frc2::SequentialCommandGroup(
    SetShooterRotation(m_shooter, ShooterState::Far),
    RunShooter(m_shooter, ShooterConstants::kShootPower)
  ).ToPtr());
  shootLowButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    ),
    frc2::InstantCommand([this]{

      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter})
  ).ToPtr());
  

  frc2::Trigger shootButton{m_ted.Button(OperatorConstants::kButtonIDRightTrigger)};
  shootButton.OnTrue(frc2::SequentialCommandGroup(
    SetShooterRotation(m_shooter, ShooterState::Close),
    RunShooter(m_shooter, ShooterConstants::kShootPower)
  ).ToPtr());
  shootButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    ),
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter})
  ).ToPtr());

  frc2::Trigger manualShootButton{m_ted.Button(OperatorConstants::kButtonIDCircle)};
  manualShootButton.OnTrue(RunShooter(m_shooter, ShooterConstants::kShootPower).ToPtr());
  manualShootButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    ),
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter})
  ).ToPtr());

  // frc2::Trigger loadTrapNoteButton{m_ted.Button(OperatorConstants::kButtonIDSquare)};
  // loadTrapNoteButton.OnTrue(frc2::InstantCommand([this]{
  //   m_shooter->SetLoaderPower(ShooterConstants::kTrapLoadLoaderPower);
  //   m_shooter->SetRollerPower(ShooterConstants::kTrapLoadRollerPower);
  // },{m_intake, m_shooter})
  //   .OnlyIf([this](){ return IsClimbState(); }));
  // loadTrapNoteButton.OnFalse(frc2::InstantCommand([this]{
  //   m_shooter->SetLoaderPower(0.0);
  //   m_shooter->SetRollerPower(0.0);
  // },{m_intake, m_shooter})
  //   .OnlyIf([this](){ return IsClimbState(); }));

  frc2::Trigger extendClimberButton{m_ted.Button(OperatorConstants::kButtonIDTriangle)};
  extendClimberButton.OnTrue(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_climber->SetServoRotation(ClimberConstants::kExtendServoAngle);
    },{m_climber}),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(0.25_s),
      RunClimber(m_climber, ClimberConstants::kRetractPower)
    ),
    RunClimber(m_climber, ClimberConstants::kExtendPower)
  )
    .OnlyIf([this](){ return IsClimbState(); }));
  extendClimberButton.OnFalse(frc2::InstantCommand([this]{
      m_climber->SetServoRotation(ClimberConstants::kRetractServoAngle);
      m_climber->SetPower(0.0);
  },{m_climber})
    .OnlyIf([this](){ return IsClimbState(); }));

  // frc2::Trigger retractClimberButton{m_ted.Button(OperatorConstants::kButtonIDX)};
  // retractClimberButton.OnTrue(RunClimber(m_climber, ClimberConstants::kRetractPower)
  //   .OnlyIf([this](){ return IsClimbState(); }));
  // retractClimberButton.OnFalse(frc2::InstantCommand([this]{
  //     m_climber->SetPower(0.0);
  // },{m_climber})
  //   .OnlyIf([this](){ return IsClimbState(); }));

  // frc2::Trigger shooterZeroButton{m_ted.Button(OperatorConstants::kButtonIDTriangle)};
  // shooterZeroButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Zero).ToPtr());

  // frc2::Trigger shooterCloseRangeButton{m_ted.Button(OperatorConstants::kButtonIDX)};
  // shooterCloseRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Close).ToPtr());

  // frc2::Trigger shooterMidRangeButton{m_ted.Button(OperatorConstants::kButtonIDCircle)};
  // shooterMidRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Mid).ToPtr());

  // frc2::Trigger shooterFarRangeButton{m_ted.Button(OperatorConstants::kButtonIDSquare)};
  // shooterFarRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Far).ToPtr());
}

void RobotContainer::RegisterAutoCommands() {
  // Start of Auto Events
  pathplanner::NamedCommands::registerCommand("AlignShooter", ShooterAutoAlign(m_shooter, m_estimator, m_vision)
        .Until([this]{ return (m_swerveDrive->GetTotalVelocity() < AutoConstants::kAlignAllowableDriveSpeed); }));

  pathplanner::NamedCommands::registerCommand("AlignSwerve", SwerveAutoAlign(m_swerveDrive, true).ToPtr());

  pathplanner::NamedCommands::registerCommand("Intake", frc2::SequentialCommandGroup(
    frc2::ParallelCommandGroup(
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetIntakeRotation(m_intake, IntakeState::Extended)
      ),
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetShooterRotation(m_shooter, ShooterState::Load)
      )
    ),
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(2.0_s),
      // RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0, ShooterEndCondition::EndOnSecondDetection),
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0, ShooterEndCondition::None),
      RunIntake(m_intake, IntakeConstants::kIntakePower)
    ),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_intake, m_shooter}),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Retracted),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    ),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(0.5_s),
      RunLoader(m_shooter, ShooterConstants::kUnloadPower, 0.0, ShooterEndCondition::None)
    )
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("ImmediateShoot", frc2::SequentialCommandGroup(
    frc2::ParallelCommandGroup(
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetIntakeRotation(m_intake, IntakeState::Extended)
      ),
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetShooterRotation(m_shooter, ShooterState::Far)
      )
    ),
    frc2::ParallelCommandGroup(
      RunIntake(m_intake, IntakeConstants::kIntakePower),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    )
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("AlignToAmp", SwerveAutoAlign(m_swerveDrive, false, 90.0_deg).ToPtr());

  pathplanner::NamedCommands::registerCommand("ScoreAmp", frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeState::Amp),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(IntakeConstants::kAmpShootTime), 
      RunIntake(m_intake, IntakeConstants::kAmpScorePower)
    ),
    SetIntakeRotation(m_intake, IntakeState::Retracted),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
    },{m_intake})
  ).ToPtr());  
  
  pathplanner::NamedCommands::registerCommand("ScoreSpeaker", frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(AutoConstants::kAutoRevUpTime),
      RunShooter(m_shooter, ShooterConstants::kShootPower)
    ),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    ),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Retracted),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    )
  ).ToPtr());

} 

void RobotContainer::CreateAutoPaths() {
  m_chooser.SetDefaultOption("N/A", nullptr);
  for (auto autoPath : AutoConstants::kAutonomousPaths) {
    m_chooser.AddOption(autoPath, new pathplanner::PathPlannerAuto(std::string{autoPath}));
  }
  m_chooser.AddOption("Dumb Two Score Mid", new TwoNoteAuto(m_swerveDrive, m_intake, m_shooter, AutoConstants::kStartingPosition));
  m_chooser.AddOption("Mobility", new MobilityAuto(m_swerveDrive));
  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_chooser.GetSelected();
}

SuperstructureState RobotContainer::GetSuperstructureState() {
  return m_superstructureState;
}

bool RobotContainer::IsClimbState() {
  return m_superstructureState == SuperstructureState::Climb;
}

void RobotContainer::SetAllCoast() {
  m_climber->SetBrakeMode(BrakeMode::Coast);
  m_intake->SetBrakeMode(BrakeMode::Coast);
  m_shooter->SetBrakeMode(BrakeMode::Coast);
  m_swerveDrive->SetBrakeMode(BrakeMode::Coast);
}

void RobotContainer::SetAllNormalBrakeMode() {
  m_climber->SetBrakeMode(BrakeMode::Default);
  m_intake->SetBrakeMode(BrakeMode::Default);
  m_shooter->SetBrakeMode(BrakeMode::Default);
  m_swerveDrive->SetBrakeMode(BrakeMode::Default);
}

BrakeMode RobotContainer::GetBrakeMode() {
  return m_climber->GetBrakeMode();
}

void RobotContainer::ConfigureSysIDBindings() {
  // System ID Bindings -- intake
  // frc2::Trigger intakeQuasiForward{m_test.Button(OperatorConstants::kButtonIDLeftTrigger)};
  // intakeQuasiForward.WhileTrue(m_intake->SysIdQuasistatic(frc2::sysid::Direction::kForward));

  // frc2::Trigger intakeQuasiReverse{m_test.Button(OperatorConstants::kButtonIDRightTrigger)};
  // intakeQuasiReverse.WhileTrue(m_intake->SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  // frc2::Trigger intakeDynamicForward{m_test.Button(OperatorConstants::kButtonIDLeftBumper)};
  // intakeDynamicForward.WhileTrue(m_intake->SysIdDynamic(frc2::sysid::Direction::kForward));

  // frc2::Trigger intakeDynamicReverse{m_test.Button(OperatorConstants::kButtonIDRightBumper)};
  // intakeDynamicReverse.WhileTrue(m_intake->SysIdDynamic(frc2::sysid::Direction::kReverse));

  // // Shooter
  // frc2::Trigger shooterQuasiForward{m_test.Button(OperatorConstants::kButtonIDX)};
  // shooterQuasiForward.WhileTrue(m_shooter->SysIdQuasistatic(frc2::sysid::Direction::kForward));

  // frc2::Trigger shooterQuasiReverse{m_test.Button(OperatorConstants::kButtonIDSquare)};
  // shooterQuasiReverse.WhileTrue(m_shooter->SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  // frc2::Trigger shooterDynamicForward{m_test.Button(OperatorConstants::kButtonIDCircle)};
  // shooterDynamicForward.WhileTrue(m_shooter->SysIdDynamic(frc2::sysid::Direction::kForward));

  // frc2::Trigger shooterDynamicReverse{m_test.Button(OperatorConstants::kButtonIDTriangle)};
  // shooterDynamicReverse.WhileTrue(m_shooter->SysIdDynamic(frc2::sysid::Direction::kReverse));

  // // Swerve drive motors
  // frc2::Trigger swerveDriveQuasiForward{m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDX)};
  // swerveDriveQuasiForward.WhileTrue(m_swerveDrive->DriveSysIdQuasistatic(frc2::sysid::Direction::kForward));

  // frc2::Trigger swerveDriveQuasiReverse{m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDSquare)};
  // swerveDriveQuasiReverse.WhileTrue(m_swerveDrive->DriveSysIdQuasistatic(frc2::sysid::Direction::kReverse));

  // frc2::Trigger swerveDriveDynamicForward{m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDTriangle)};
  // swerveDriveDynamicForward.WhileTrue(m_swerveDrive->DriveSysIdDynamic(frc2::sysid::Direction::kForward));

  // frc2::Trigger swerveDriveDynamicReverse{m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDCircle)};
  // swerveDriveDynamicReverse.WhileTrue(m_swerveDrive->DriveSysIdDynamic(frc2::sysid::Direction::kReverse));

  // // Swerve angle motors
  // frc2::Trigger swerveAngleQuasiForward{m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDX)};
  // swerveAngleQuasiForward.WhileTrue(m_swerveDrive->AngleSysIdQuasistatic(frc2::sysid::Direction::kForward));

  // frc2::Trigger swerveAngleQuasiReverse{m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDSquare)};
  // swerveAngleQuasiReverse.WhileTrue(m_swerveDrive->AngleSysIdQuasistatic(frc2::sysid::Direction::kReverse));

  // frc2::Trigger swerveAngleDynamicForward{m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDTriangle)};
  // swerveAngleDynamicForward.WhileTrue(m_swerveDrive->AngleSysIdDynamic(frc2::sysid::Direction::kForward));

  // frc2::Trigger swerveAngleDynamicReverse{m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDCircle)};
  // swerveAngleDynamicReverse.WhileTrue(m_swerveDrive->AngleSysIdDynamic(frc2::sysid::Direction::kReverse));
}

// no matter how nice ethan seems he will slap you with a piece of chicken and eat you in a bucket