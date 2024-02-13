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
  
  m_intake->SetDefaultCommand(frc2::RunCommand([this] {
    m_intake->SetRotationPower(m_ted.GetRawAxis(OperatorConstants::kAxisLeftStickY));
  },{m_intake}).ToPtr());

  m_shooter->SetDefaultCommand(frc2::RunCommand([this]{
    m_shooter->SetRotationPower(m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY));
    // May need to change joystick axis
  }, {m_shooter}).ToPtr());
  
  // Configure the button bindings
  ConfigureDriverBindings();
  ConfigureCoDriverBindings();
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

  frc2::Trigger intakeButton{m_bill.Button(OperatorConstants::kButtonIDLeftBumper)};
  intakeButton.OnTrue(RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0).ToPtr());
  intakeButton.OnFalse(frc2::InstantCommand([this]{
    m_intake->SetRollerPower(0.0);
    m_intake->SetRotationPower(0.0);
  },{m_intake}).ToPtr());

  // intakeButton.OnTrue(FullIntake(m_intake, IntakeConstants::kIntakePower, IntakeConstants::kExtendTarget).ToPtr());

  frc2::Trigger retractIntakeButton{m_bill.Button(OperatorConstants::kButtonIDX)};
  retractIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget).ToPtr());

  frc2::Trigger ampScoreIntakeButton{m_bill.Button(OperatorConstants::kButtonIDCircle)};
    ampScoreIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeConstants::kAmpTarget).ToPtr());
    ampScoreIntakeButton.OnFalse(frc2::SequentialCommandGroup(
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(2.0_s), 
        RunIntake(m_intake, -1.0 * IntakeConstants::kIntakePower, 0)
      ),
      SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
    ).ToPtr());
  
  // frc2::Trigger ampScoreIntakeButton{m_bill.Button(OperatorConstants::kButtonIDCircle)};
  // ampScoreIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeConstants::kAmpTarget).ToPtr());

  frc2::Trigger extendIntakeButton{m_bill.Button(OperatorConstants::kButtonIDTriangle)};
  extendIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeConstants::kExtendTarget).ToPtr());

  frc2::Trigger resetPoseButton{m_bill.Button(OperatorConstants::kButtonIDCreate)};
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    // m_swerveDrive->ResetGyroscope();
    m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}}, true);
  },{m_swerveDrive}).ToPtr());

  frc2::Trigger resetSpeakerPoseButton{m_bill.Button(OperatorConstants::kButtonIDMenu)};
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    m_swerveDrive->SetPose(frc::Pose2d{0.92_m, 0.0_m, frc::Rotation2d{180.0_deg}}, true);
  },{m_swerveDrive}).ToPtr());

  frc2::Trigger alignSpeakerButton{m_bill.Button(OperatorConstants::kButtonIDTouchpad)};
  alignSpeakerButton.ToggleOnTrue(Drive(&m_bill, m_swerveDrive, false, true, true).ToPtr());
  
  frc2::Trigger toggleSlowModeButton{m_bill.Button(OperatorConstants::kButtonIDRightBumper)};
  toggleSlowModeButton.OnTrue(frc2::InstantCommand([this]{ m_swerveDrive->ToggleSlowMode(); },{m_swerveDrive}).ToPtr());

  frc2::Trigger extendClimbButton{m_bill.Button(OperatorConstants::kButtonIDLeftTrigger)};
  extendClimbButton.OnTrue(frc2::InstantCommand([this]{ 
    m_climber->SetPower(ClimberConstants::kExtendMotorSpeed);
    m_climber->SetServoRotation(ClimberConstants::kExtendServoAngle);
  },{m_climber}).ToPtr());
  extendClimbButton.OnFalse(frc2::InstantCommand([this]{
    m_climber->SetPower(0.0);
    m_climber->SetServoRotation(ClimberConstants::kRetractServoAngle);
  },{m_climber}).ToPtr());

  frc2::Trigger retractClimbButton{m_bill.Button(OperatorConstants::kButtonIDRightTrigger)};
  retractClimbButton.OnTrue(frc2::InstantCommand([this]{
    m_climber->SetPower(ClimberConstants::kRetractMotorSpeed);
  },{m_climber}).ToPtr());
  retractClimbButton.OnFalse(frc2::InstantCommand([this]{
    m_climber->SetPower(0.0);
  },{m_climber}).ToPtr());
}

void RobotContainer::ConfigureCoDriverBindings() {
  // Ted controls
  frc2::Trigger shootButton{m_ted.Button(OperatorConstants::kButtonIDRightTrigger)};
  shootButton.OnTrue(RunShooter(m_shooter, ShooterConstants::kShootPower).ToPtr());
  shootButton.OnFalse(frc2::InstantCommand([this]{
    m_shooter->SetRollerPower(0.0);
  },{m_shooter}).ToPtr());

  frc2::Trigger shooterAlignButton{m_ted.Button(OperatorConstants::kButtonIDLeftTrigger)};
  shooterAlignButton.OnTrue(ShooterAutoAlign(m_shooter, m_estimator, m_vision).ToPtr());
  shooterAlignButton.OnFalse(frc2::InstantCommand([this]{
    m_shooter->SetRotationPower(0.0);
  },{m_shooter}).ToPtr());

  frc2::Trigger shooterCloseRangeButton{m_ted.Button(OperatorConstants::kButtonIDX)};
  shooterCloseRangeButton.OnTrue(SetShooterRotation(m_shooter, units::degree_t{ShooterConstants::kCloseTarget}).ToPtr());

  frc2::Trigger shooterMidRangeButton{m_ted.Button(OperatorConstants::kButtonIDCircle)};
  shooterMidRangeButton.OnTrue(SetShooterRotation(m_shooter, units::degree_t{ShooterConstants::kMidTarget}).ToPtr());

  frc2::Trigger shooterFarRangeButton{m_ted.Button(OperatorConstants::kButtonIDTriangle)};
  shooterFarRangeButton.OnTrue(SetShooterRotation(m_shooter, units::degree_t{ShooterConstants::kFarTarget}).ToPtr());

  frc2::Trigger loadButton{m_ted.Button(OperatorConstants::kButtonIDLeftBumper)};
  loadButton.OnTrue(frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeConstants::kExtendTarget),
    SetShooterRotation(m_shooter, ShooterConstants::kLoadAngle),
    frc2::ParallelDeadlineGroup(
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0),
      RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0)
    ),
    SetShooterRotation(m_shooter, units::degree_t{ShooterConstants::kRetractTarget}),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
  ).ToPtr());
  loadButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_intake, m_shooter}),
    SetShooterRotation(m_shooter, units::degree_t{ShooterConstants::kRetractTarget}),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
  ).ToPtr());

  frc2::Trigger directShooterLoadButton{m_ted.Button(OperatorConstants::kButtonIDRightBumper)};
  directShooterLoadButton.OnTrue(frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeConstants::kAmpTarget),
    SetShooterRotation(m_shooter, ShooterConstants::kDirectLoadAngle),
    RunLoader(m_shooter, ShooterConstants::kDirectLoadPower, ShooterConstants::kDirectLoadPower),
    SetShooterRotation(m_shooter, units::degree_t{ShooterConstants::kRetractTarget}),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
  ).ToPtr());
  directShooterLoadButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}),
    SetShooterRotation(m_shooter, units::degree_t{ShooterConstants::kRetractTarget}),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
  ).ToPtr());
}

void RobotContainer::RegisterAutoCommands() {
  // Start of Auto Events
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.emplace_back(SetIntakeRotation(m_intake, IntakeConstants::kAmpTarget).ToPtr().Unwrap());
  commands.emplace_back(ShooterAutoAlign(m_shooter, m_estimator, m_vision)
        .Until([this]{ return (m_swerveDrive->GetTotalVelocity() < 0.25_mps); }).Unwrap());
  pathplanner::NamedCommands::registerCommand("AlignShooter", frc2::SequentialCommandGroup(std::move(commands)).ToPtr());

  pathplanner::NamedCommands::registerCommand("AlignSwerve", SwerveAutoAlign(m_swerveDrive, true).ToPtr());

  pathplanner::NamedCommands::registerCommand("Intake", frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(3.0_s),
      FullIntake(m_intake, IntakeConstants::kIntakePower, IntakeConstants::kExtendTarget)
    ),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
    },{m_intake}),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("ImmediateShoot", frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeConstants::kExtendTarget),
    RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0),
    SetShooterRotation(m_shooter, ShooterConstants::kImmediateShootAngle),
    RunShooter(m_shooter, ShooterConstants::kShootPower)
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("Load", frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeConstants::kExtendTarget),
    SetShooterRotation(m_shooter, ShooterConstants::kLoadAngle),
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(2.0_s),
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0),
      RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0)
    ),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
      m_shooter->SetRollerPower(0.0);
    },{m_intake, m_shooter}),
    SetShooterRotation(m_shooter, units::degree_t{ShooterConstants::kRetractTarget}),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("AlignToAmp", SwerveAutoAlign(m_swerveDrive, false, 90_deg).ToPtr());

  pathplanner::NamedCommands::registerCommand("ScoreAmp", frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeConstants::kAmpTarget),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(2.0_s), 
      RunIntake(m_intake, -1 * IntakeConstants::kIntakePower, 0)
    ),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
    },{m_intake})
  ).ToPtr());  
  
  pathplanner::NamedCommands::registerCommand("ScoreSpeaker", frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(1.0_s),
      RunShooter(m_shooter, ShooterConstants::kShootPower)
    ),
    SetShooterExtension(m_shooter, ShooterConstants::kRetractTarget),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
  ).ToPtr());
} 

void RobotContainer::CreateAutoPaths() {
  m_chooser.SetDefaultOption("N/A", nullptr);
  m_chooser.AddOption("Score 1 - Top", new pathplanner::PathPlannerAuto("Score 1 - Top"));
  m_chooser.AddOption("Score 1 - Mid", new pathplanner::PathPlannerAuto("Score 1 - Mid"));
  m_chooser.AddOption("Score 1 - Bottom", new pathplanner::PathPlannerAuto("Score 1 - Bottom"));
  m_chooser.AddOption("Score 2 - Top", new pathplanner::PathPlannerAuto("Score 2 - Top"));
  m_chooser.AddOption("Score 2 - Mid", new pathplanner::PathPlannerAuto("Score 2 - Mid"));
  m_chooser.AddOption("Score 2 - Mid - Amp", new pathplanner::PathPlannerAuto("Score 2 - Mid - Amp"));
  m_chooser.AddOption("Score 2 - Bottom", new pathplanner::PathPlannerAuto("Score 2 - Bottom"));
  m_chooser.AddOption("Score 3 - Top 1", new pathplanner::PathPlannerAuto("Score 3 - Top 1"));
  m_chooser.AddOption("Score 3 - Top - Amp", new pathplanner::PathPlannerAuto("Score 3 - Top - Amp"));
  m_chooser.AddOption("Score 3 - Top 1 - Amp x2", new pathplanner::PathPlannerAuto("Score 3 - Top 1 - Amp x2"));
  m_chooser.AddOption("Score 3 - Mid 2", new pathplanner::PathPlannerAuto("Score 3 - Mid 2"));
  m_chooser.AddOption("Score 3 - Mid 3 - Under", new pathplanner::PathPlannerAuto("Score 3 - Mid 3 - Under"));
  m_chooser.AddOption("Score 3 - Bottom 5", new pathplanner::PathPlannerAuto("Score 3 - Bottom 5"));
  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_chooser.GetSelected();
}

void RobotContainer::ConfigureSysIDBindings() {
  (m_test.Button(OperatorConstants::kButtonIDLeftTrigger) && m_test.Button(OperatorConstants::kButtonIDX))
    .WhileTrue(m_intake->SysIdQuasistatic(frc2::sysid::Direction::kForward));
  (m_test.Button(OperatorConstants::kButtonIDLeftTrigger) && m_test.Button(OperatorConstants::kButtonIDSquare))
    .WhileTrue(m_intake->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  (m_test.Button(OperatorConstants::kButtonIDLeftTrigger) && m_test.Button(OperatorConstants::kButtonIDTriangle))
    .WhileTrue(m_intake->SysIdDynamic(frc2::sysid::Direction::kForward));
  (m_test.Button(OperatorConstants::kButtonIDLeftTrigger) && m_test.Button(OperatorConstants::kButtonIDCircle))
    .WhileTrue(m_intake->SysIdDynamic(frc2::sysid::Direction::kReverse));

  (m_test.Button(OperatorConstants::kButtonIDRightTrigger) && m_test.Button(OperatorConstants::kButtonIDX))
    .WhileTrue(m_shooter->SysIdQuasistatic(frc2::sysid::Direction::kForward));
  (m_test.Button(OperatorConstants::kButtonIDRightTrigger) && m_test.Button(OperatorConstants::kButtonIDSquare))
    .WhileTrue(m_shooter->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  (m_test.Button(OperatorConstants::kButtonIDRightTrigger) && m_test.Button(OperatorConstants::kButtonIDTriangle))
    .WhileTrue(m_shooter->SysIdDynamic(frc2::sysid::Direction::kForward));
  (m_test.Button(OperatorConstants::kButtonIDRightTrigger) && m_test.Button(OperatorConstants::kButtonIDCircle))
    .WhileTrue(m_shooter->SysIdDynamic(frc2::sysid::Direction::kReverse));

  // System ID Bindings
  (m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDX))
    .WhileTrue(m_swerveDrive->DriveSysIdQuasistatic(frc2::sysid::Direction::kForward));
  (m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDSquare))
    .WhileTrue(m_swerveDrive->DriveSysIdQuasistatic(frc2::sysid::Direction::kReverse));
  (m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDTriangle))
    .WhileTrue(m_swerveDrive->DriveSysIdDynamic(frc2::sysid::Direction::kForward));
  (m_test.Button(OperatorConstants::kButtonIDLeftBumper) && m_test.Button(OperatorConstants::kButtonIDCircle))
    .WhileTrue(m_swerveDrive->DriveSysIdDynamic(frc2::sysid::Direction::kReverse));

  (m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDX))
    .WhileTrue(m_swerveDrive->AngleSysIdQuasistatic(frc2::sysid::Direction::kForward));
  (m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDSquare))
    .WhileTrue(m_swerveDrive->AngleSysIdQuasistatic(frc2::sysid::Direction::kReverse));
  (m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDTriangle))
    .WhileTrue(m_swerveDrive->AngleSysIdDynamic(frc2::sysid::Direction::kForward));
  (m_test.Button(OperatorConstants::kButtonIDRightBumper) && m_test.Button(OperatorConstants::kButtonIDCircle))
    .WhileTrue(m_swerveDrive->AngleSysIdDynamic(frc2::sysid::Direction::kReverse));
}

// no matter how nice ethan seems he will slap you with a piece of chicken and eat you in a bucket with skibidi toilet