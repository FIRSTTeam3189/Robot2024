// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
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
    m_intake->SetRollerPower(m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY));
  },{m_intake}).ToPtr());
  // Configure the button bindings
  ConfigureDriverBindings();
  ConfigureCoDriverBindings();
  RegisterAutoCommands();
}

void RobotContainer::ConfigureDriverBindings() {
   // Bill controls
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
    m_swerveDrive->ResetGyroscope();
    m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}});
  },{m_swerveDrive}).ToPtr());

  frc2::Trigger alignSpeakerButton{m_bill.Button(OperatorConstants::kButtonIDTouchpad)};
  alignSpeakerButton.ToggleOnTrue(Drive(&m_bill, m_swerveDrive, m_isSpecialHeadingMode, true, true).ToPtr());
  
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

  frc2::Trigger shooterCloseRangeButton{m_ted.Button(OperatorConstants::kButtonIDX)};
  shooterCloseRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterConstants::kCloseTarget).ToPtr());

  frc2::Trigger shooterMidRangeButton{m_ted.Button(OperatorConstants::kButtonIDCircle)};
  shooterMidRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterConstants::kMidTarget).ToPtr());

  frc2::Trigger shooterFarRangeButton{m_ted.Button(OperatorConstants::kButtonIDTriangle)};
  shooterFarRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterConstants::kFarTarget).ToPtr());

  frc2::Trigger loadButton{m_ted.Button(OperatorConstants::kButtonIDLeftBumper)};
  loadButton.OnTrue(frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeConstants::kExtendTarget),
    SetShooterRotation(m_shooter, ShooterConstants::kLoadAngle),
    frc2::ParallelDeadlineGroup(
      RunLoader(m_shooter, ShooterConstants::kLoadPower),
      RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0)
    ),
    SetShooterRotation(m_shooter, ShooterConstants::kRetractTarget),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
  ).ToPtr());
  loadButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
      m_shooter->SetRollerPower(0.0);
    },{m_intake, m_shooter}),
    SetShooterRotation(m_shooter, ShooterConstants::kRetractTarget),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
  ).ToPtr());
}

void RobotContainer::RegisterAutoCommands(){

  //Start of Auto Events
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

  pathplanner::NamedCommands::registerCommand("Load", frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeConstants::kExtendTarget),
    SetShooterRotation(m_shooter, ShooterConstants::kLoadAngle),
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(2.0_s),
      RunLoader(m_shooter, ShooterConstants::kLoadPower),
      RunIntake(m_intake, IntakeConstants::kIntakePower, 0.0)
    ),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
      m_shooter->SetRollerPower(0.0);
    },{m_intake, m_shooter}),
    SetShooterRotation(m_shooter, ShooterConstants::kRetractTarget),
    SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget)
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("ampScoreIntake", frc2::SequentialCommandGroup(
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

  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.emplace_back(SetIntakeRotation(m_intake, IntakeConstants::kAmpTarget).ToPtr().Unwrap());
  commands.emplace_back(ShooterAutoAlign(m_shooter, m_estimator, m_vision)
        .Until([this]{ return (m_swerveDrive->GetTotalVelocity() < 0.25_mps); }).Unwrap());
  commands.emplace_back(frc2::ParallelDeadlineGroup(
    frc2::WaitCommand(1.0_s),
    RunShooter(m_shooter, ShooterConstants::kShootPower)
  ).ToPtr().Unwrap());
  commands.emplace_back(SetShooterExtension(m_shooter, ShooterConstants::kRetractTarget).ToPtr().Unwrap());
  commands.emplace_back(SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget).ToPtr().Unwrap());
  
  pathplanner::NamedCommands::registerCommand("speakerScore", frc2::SequentialCommandGroup(std::move(commands)).ToPtr());
} 

void RobotContainer::CreateAutoPaths() {
  m_chooser.AddOption("Test Auto", new TestAuto("Test Auto", m_swerveDrive));
}



frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_chooser.GetSelected();
}

