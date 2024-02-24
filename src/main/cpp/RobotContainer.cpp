// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  RegisterAutoCommands();
  
  // Initialize all of your commands and subsystems here
  // Runs shooter and shooter angle motor continuously
  // m_shooter->SetDefaultCommand(frc2::RunCommand([this] { 
  //   m_shooter->SetRollerPower(m_ted.GetRawAxis(OperatorConstants::kAxisLeftStickY)); 
  //   m_shooter->SetRotationPower(m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY));
  // },{m_shooter}).ToPtr());

  m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_isSpecialHeadingMode));
    frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
  
  // m_intake->SetDefaultCommand(frc2::RunCommand([this] {
  //   m_intake->SetRotationPower(m_ted.GetRawAxis(OperatorConstants::kAxisLeftStickY));
  // },{m_intake}).ToPtr());

  m_shooter->SetDefaultCommand(frc2::RunCommand([this]{
    auto input = m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY);
    if (fabs(input) > 0.05)
      m_shooter->SetRotationPower(-m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY));
    else
      m_shooter->SetRotationPower(0.02);
    // May need to change joystick axis
  }, {m_shooter}).ToPtr());
  
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
      m_isSpecialHeadingMode = !m_isSpecialHeadingMode;
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_isSpecialHeadingMode, m_isFieldRelative));
    },{m_swerveDrive}).ToPtr()
  );

  frc2::Trigger toggleFieldRelativeButton{m_bill.Button(OperatorConstants::kButtonIDLeftStick)};
  toggleFieldRelativeButton.OnTrue(
    frc2::InstantCommand([this]{
      m_isFieldRelative = !m_isFieldRelative;
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_isSpecialHeadingMode, m_isFieldRelative));
    },{m_swerveDrive}).ToPtr()
  );

  frc2::Trigger intakeButton{m_bill.Button(OperatorConstants::kButtonIDRightBumper)};
  intakeButton.OnTrue(FullIntake(m_intake, IntakeConstants::kIntakePower, IntakeConstants::kExtendTarget).ToPtr());
  intakeButton.OnFalse(frc2::InstantCommand([this]{
    m_intake->SetRollerPower(0.0);
    m_intake->SetRotation(IntakeConstants::kRetractTarget);
  },{m_intake}).ToPtr());

  frc2::Trigger driverLoadButton{m_bill.Button(OperatorConstants::kButtonIDLeftBumper)};
  driverLoadButton.OnTrue(frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(1.25_s),
      SetIntakeRotation(m_intake, IntakeState::Extended),
      SetShooterRotation(m_shooter, ShooterState::Load)
    ),
    frc2::ParallelCommandGroup(
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0),
      RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0)
    )
  ).ToPtr());
  driverLoadButton.OnFalse(frc2::SequentialCommandGroup(
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

  frc2::Trigger retractIntakeButton{m_bill.Button(OperatorConstants::kButtonIDX)};
  retractIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeState::Retracted).ToPtr());

  frc2::Trigger ampScoreIntakeButton{m_bill.Button(OperatorConstants::kButtonIDRightTrigger)};
  ampScoreIntakeButton.OnTrue(frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeState::Amp)
  ).ToPtr());
  ampScoreIntakeButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(2.0_s), 
      RunIntake(m_intake, IntakeConstants::kAmpScorePower, 0)
    ),
    SetIntakeRotation(m_intake, IntakeState::Retracted)
  ).ToPtr());

  frc2::Trigger extendIntakeButton{m_bill.Button(OperatorConstants::kButtonIDTriangle)};
  extendIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeState::Extended).ToPtr());

  frc2::Trigger resetPoseButton{m_bill.Button(OperatorConstants::kButtonIDCreate)};
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    // m_swerveDrive->ResetGyroscope();
    m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}}, false);
  },{m_swerveDrive}).ToPtr());

  frc2::Trigger resetSpeakerPoseButton{m_bill.Button(OperatorConstants::kButtonIDMenu)};
  resetSpeakerPoseButton.OnTrue(frc2::InstantCommand([this]{
    if (frc::DriverStation::GetAlliance()) {
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        m_swerveDrive->SetPose(frc::Pose2d{0.92_m, 5.50_m, frc::Rotation2d{0.0_deg}}, false);
      else
        m_swerveDrive->SetPose(frc::Pose2d{15.579_m, 5.50_m, frc::Rotation2d{180.0_deg}}, false);
    }
  },{m_swerveDrive}).ToPtr());

  frc2::Trigger alignSpeakerButton{m_bill.Button(OperatorConstants::kButtonIDLeftTrigger)};
  alignSpeakerButton.ToggleOnTrue(Drive(&m_bill, m_swerveDrive, false, true, true).ToPtr());
  
  // frc2::Trigger toggleSlowModeButton{m_bill.Button(OperatorConstants::kButtonIDRightBumper)};
  // toggleSlowModeButton.OnTrue(frc2::InstantCommand([this]{ m_swerveDrive->ToggleSlowMode(); },{m_swerveDrive}).ToPtr());

  // frc2::Trigger extendClimbButton{m_bill.Button(OperatorConstants::kButtonIDLeftTrigger)};
  // extendClimbButton.OnTrue(frc2::InstantCommand([this]{ 
  //   m_climber->SetPower(ClimberConstants::kExtendMotorSpeed);
  //   m_climber->SetServoRotation(ClimberConstants::kExtendServoAngle);
  // },{m_climber}).ToPtr());
  // extendClimbButton.OnFalse(frc2::InstantCommand([this]{
  //   m_climber->SetPower(0.0);
  //   m_climber->SetServoRotation(ClimberConstants::kRetractServoAngle);
  // },{m_climber}).ToPtr());

  // frc2::Trigger retractClimbButton{m_bill.Button(OperatorConstants::kButtonIDRightTrigger)};
  // retractClimbButton.OnTrue(frc2::InstantCommand([this]{
  //   m_climber->SetPower(ClimberConstants::kRetractMotorSpeed);
  // },{m_climber}).ToPtr());
  // retractClimbButton.OnFalse(frc2::InstantCommand([this]{
  //   m_climber->SetPower(0.0);
  // },{m_climber}).ToPtr());
}

void RobotContainer::ConfigureCoDriverBindings() {
  // Ted controls
  frc2::Trigger shootButton{m_ted.Button(OperatorConstants::kButtonIDRightTrigger)};
  shootButton.OnTrue(frc2::SequentialCommandGroup(
    SetShooterRotation(m_shooter, ShooterState::Close),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kRevUpTime),
      RunShooter(m_shooter, ShooterConstants::kShootPower)
    ),
    RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
  ).ToPtr());
  shootButton.OnFalse(frc2::InstantCommand([this]{
    m_shooter->SetRollerPower(0.0);
    m_shooter->SetLoaderPower(0.0);
  },{m_shooter}).ToPtr());

  // frc2::Trigger loadClimbPieceButton{m_ted.Button(OperatorConstants::kButtonIDMenu)};
  // loadClimbPieceButton.OnTrue(frc2::SequentialCommandGroup(
  //   TransferLoader(m_shooter, ShooterConstants::kLoadPower, 0.0)
  // ).ToPtr());

  // frc2::Trigger extendShooterButton{m_ted.Button(OperatorConstants::kButtonIDMicrophone)};
  // extendShooterButton.OnTrue(frc2::SequentialCommandGroup(
  //     SetIntakeRotation(m_intake, IntakeState::Retracted),
  //     SetShooterRotation(m_shooter, ShooterConstants::kTrapExtensionAngle),
  //     SetShooterExtension(m_shooter, ShooterConstants::kTrapExtension)
  //   ).ToPtr());

  frc2::Trigger shooterAlignButton{m_ted.Button(OperatorConstants::kButtonIDTouchpad)};
  shooterAlignButton.OnTrue(ShooterAutoAlign(m_shooter, m_estimator, m_vision).ToPtr());
  shooterAlignButton.OnFalse(frc2::InstantCommand([this]{
    m_shooter->SetRotationPower(0.0);
  },{m_shooter}).ToPtr());

  frc2::Trigger shooterCloseRangeButton{m_ted.Button(OperatorConstants::kButtonIDX)};
  shooterCloseRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Close).ToPtr());

  frc2::Trigger shooterMidRangeButton{m_ted.Button(OperatorConstants::kButtonIDCircle)};
  shooterMidRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Mid).ToPtr());

  // frc2::Trigger shooterFarRangeButton{m_ted.Button(OperatorConstants::kButtonIDTriangle)};
  // shooterFarRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Far).ToPtr());

  frc2::Trigger manualShootButton{m_ted.Button(OperatorConstants::kButtonIDTriangle)};
  manualShootButton.OnTrue(frc2::SequentialCommandGroup(
  frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kRevUpTime),
      RunShooter(m_shooter, ShooterConstants::kShootPower)
    ),
    RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
  ).ToPtr());
  manualShootButton.OnFalse(frc2::InstantCommand([this]{
    m_shooter->SetRollerPower(0.0);
    m_shooter->SetLoaderPower(0.0);
  },{m_shooter}).ToPtr());

  frc2::Trigger loadButton{m_ted.Button(OperatorConstants::kButtonIDLeftBumper)};
  // loadButton.OnTrue(frc2::SequentialCommandGroup(
  //   SetIntakeRotation(m_intake, IntakeState::Extended),
  //   SetShooterRotation(m_shooter, ShooterState::Load),
  //   frc2::ParallelDeadlineGroup(
  //     RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0),
  //     RunIntake(m_intake, IntakeConstants::kIntakePower / 2.0, 0.0)
  //   ),
  //   SetShooterRotation(m_shooter, ShooterState::Retracted),
  //   SetIntakeRotation(m_intake, IntakeState::Retracted)
  // ).ToPtr());

  loadButton.OnTrue(frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(1.25_s),
      SetIntakeRotation(m_intake, IntakeState::Extended),
      SetShooterRotation(m_shooter, ShooterState::Load)
    ),
    frc2::ParallelCommandGroup(
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0),
      RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0)
    )
  ).ToPtr());
  loadButton.OnFalse(frc2::SequentialCommandGroup(
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

  frc2::Trigger unloadButton{m_ted.Button(OperatorConstants::kButtonIDRightBumper)};
  unloadButton.OnTrue(RunLoader(m_shooter, ShooterConstants::kUnloadPower, 0.0).ToPtr());
  unloadButton.OnFalse(
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}).ToPtr()
  );

  frc2::Trigger directShooterLoadButton{m_ted.Button(OperatorConstants::kButtonIDLeftTrigger)};
  directShooterLoadButton.OnTrue(frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeState::Amp),
    SetShooterRotation(m_shooter, ShooterState::Load),
    RunLoader(m_shooter, ShooterConstants::kDirectLoadPower, ShooterConstants::kDirectLoadPower),
    SetShooterRotation(m_shooter, ShooterState::Retracted),
    SetIntakeRotation(m_intake, IntakeState::Retracted)
  ).ToPtr());
  directShooterLoadButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}),
    SetShooterRotation(m_shooter, ShooterState::Retracted),
    SetIntakeRotation(m_intake, IntakeState::Retracted)
  ).ToPtr());
}

void RobotContainer::RegisterAutoCommands() {
  // Start of Auto Events
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.emplace_back(ShooterAutoAlign(m_shooter, m_estimator, m_vision)
        .Until([this]{ return (m_swerveDrive->GetTotalVelocity() < 0.25_mps); }).Unwrap());
  pathplanner::NamedCommands::registerCommand("AlignShooter", frc2::SequentialCommandGroup(std::move(commands)).ToPtr());

  pathplanner::NamedCommands::registerCommand("AlignSwerve", SwerveAutoAlign(m_swerveDrive, true).ToPtr());

  pathplanner::NamedCommands::registerCommand("Intake", frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(1.25_s),
      SetIntakeRotation(m_intake, IntakeState::Extended),
      SetShooterRotation(m_shooter, ShooterState::Load)
    ),
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(2.0_s),
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0),
      RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0)
    ),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_intake, m_shooter}),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Retracted),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    )
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("ImmediateShoot", frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(1.25_s),
      SetIntakeRotation(m_intake, IntakeState::Extended),
      SetShooterRotation(m_shooter, ShooterState::Far)
    ),
    frc2::ParallelCommandGroup(
      RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    )
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("Load", frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeState::Extended),
    SetShooterRotation(m_shooter, ShooterState::Load),
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(2.0_s),
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0),
      RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0)
    ),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
      m_shooter->SetRollerPower(0.0);
    },{m_intake, m_shooter}),
    SetShooterRotation(m_shooter, ShooterState::Retracted),
    SetIntakeRotation(m_intake, IntakeState::Retracted)
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("AlignToAmp", SwerveAutoAlign(m_swerveDrive, false, 90_deg).ToPtr());

  pathplanner::NamedCommands::registerCommand("ScoreAmp", frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeState::Amp),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(2.0_s), 
      RunIntake(m_intake, IntakeConstants::kAmpScorePower, 0)
    ),
    SetIntakeRotation(m_intake, IntakeState::Retracted),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
    },{m_intake})
  ).ToPtr());  
  
  pathplanner::NamedCommands::registerCommand("ScoreSpeaker", frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kRevUpTime),
      RunShooter(m_shooter, ShooterConstants::kShootPower)
    ),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(1.0_s),
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
  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_chooser.GetSelected();
}

void RobotContainer::ConfigureSysIDBindings() {
  // System ID Bindings
  // m_test.Button(OperatorConstants::kButtonIDLeftTrigger).WhileTrue(m_intake->SysIdQuasistatic(frc2::sysid::Direction::kForward));

  frc2::Trigger ObamaButton{m_test.Button(OperatorConstants::kButtonIDLeftTrigger)};
  ObamaButton.WhileTrue(m_intake->SysIdQuasistatic(frc2::sysid::Direction::kForward));

  frc2::Trigger Obama1Button{m_test.Button(OperatorConstants::kButtonIDRightTrigger)};
  Obama1Button.WhileTrue(m_intake->SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  frc2::Trigger Obama2Button{m_test.Button(OperatorConstants::kButtonIDLeftBumper)};
  Obama2Button.WhileTrue(m_intake->SysIdDynamic(frc2::sysid::Direction::kForward));

  frc2::Trigger Obama3Button{m_test.Button(OperatorConstants::kButtonIDRightBumper)};
  Obama3Button.WhileTrue(m_intake->SysIdDynamic(frc2::sysid::Direction::kReverse));

  frc2::Trigger Obama4Button{m_test.Button(OperatorConstants::kButtonIDX)};
  Obama4Button.WhileTrue(m_shooter->SysIdQuasistatic(frc2::sysid::Direction::kForward));

  frc2::Trigger Obama5Button{m_test.Button(OperatorConstants::kButtonIDSquare)};
  Obama5Button.WhileTrue(m_shooter->SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  frc2::Trigger Obama6Button{m_test.Button(OperatorConstants::kButtonIDCircle)};
  Obama6Button.WhileTrue(m_shooter->SysIdDynamic(frc2::sysid::Direction::kForward));

  frc2::Trigger Obama7Button{m_test.Button(OperatorConstants::kButtonIDTriangle)};
  Obama7Button.WhileTrue(m_shooter->SysIdDynamic(frc2::sysid::Direction::kReverse));

  frc2::Trigger Obama8Button{m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDX)};
  Obama8Button.WhileTrue(m_swerveDrive->DriveSysIdQuasistatic(frc2::sysid::Direction::kForward));

  frc2::Trigger Obama9Button{m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDSquare)};
  Obama9Button.WhileTrue(m_swerveDrive->DriveSysIdQuasistatic(frc2::sysid::Direction::kReverse));

  frc2::Trigger Obama10Button{m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDTriangle)};
  Obama10Button.WhileTrue(m_swerveDrive->DriveSysIdDynamic(frc2::sysid::Direction::kForward));

  // frc2::Trigger Obama11Button{m_test.Button(OperatorConstants::kButtonIDLeftBumper) && };
  // Obama11Button.WhileTrue(m_swerveDrive->DriveSysIdDynamic(frc2::sysid::Direction::kReverse));

  frc2::Trigger Obama12Button{m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDX)};
  Obama12Button.WhileTrue(m_swerveDrive->AngleSysIdQuasistatic(frc2::sysid::Direction::kForward));

  frc2::Trigger Obama13Button{m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDSquare)};
  Obama13Button.WhileTrue(m_swerveDrive->AngleSysIdQuasistatic(frc2::sysid::Direction::kReverse));

  frc2::Trigger Obama14Button{m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDTriangle)};
  Obama14Button.WhileTrue(m_swerveDrive->AngleSysIdDynamic(frc2::sysid::Direction::kReverse));

  frc2::Trigger Obama15Button{m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDCircle)};
  Obama15Button.WhileTrue(m_swerveDrive->AngleSysIdDynamic(frc2::sysid::Direction::kReverse));
  
}

// no matter how nice ethan seems he will slap you with a piece of chicken and eat you in a bucket