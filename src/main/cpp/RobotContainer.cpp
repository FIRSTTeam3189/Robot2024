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

  m_swerveDrive->SetDefaultCommand(JoystickDrive(&m_bill, m_swerveDrive, m_isSpecialHeadingMode));
  
  m_intake->SetDefaultCommand(frc2::RunCommand([this] {
    m_intake->SetRotationPower(m_ted.GetRawAxis(OperatorConstants::kAxisLeftStickY));
    m_intake->SetRollerPower(m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY));
  },{m_intake}).ToPtr());
  // Configure the button bindings
  ConfigureDriverBindings();
  ConfigureCoDriverBindings();
}

void RobotContainer::ConfigureDriverBindings() {
   // Bill controls
  frc2::Trigger intakeButton{m_bill.Button(OperatorConstants::kButtonIDLeftBumper)};
  intakeButton.OnTrue(RunIntake(m_intake, 0.5, 0.0).ToPtr());
  intakeButton.OnFalse(frc2::InstantCommand([this]{
    m_intake->SetRollerPower(0.0);
    m_intake->SetRotationPower(0.0);
  },{m_intake}).ToPtr());

  intakeButton.OnTrue(frc2::ParallelCommandGroup(
    FullIntake(m_intake, IntakeConstants::kIntakePower, IntakeConstants::kExtendTarget)
  ).ToPtr());

  frc2::Trigger retractIntakeButton{m_bill.Button(OperatorConstants::kButtonIDX)};
  retractIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeConstants::kRetractTarget).ToPtr());

  frc2::Trigger ampScoreIntakeButton{m_bill.Button(OperatorConstants::kButtonIDCircle)};
  ampScoreIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeConstants::kAmpTarget).ToPtr());

  frc2::Trigger extendIntakeButton{m_bill.Button(OperatorConstants::kButtonIDTriangle)};
  extendIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeConstants::kExtendTarget).ToPtr());

  frc2::Trigger resetPoseButton{m_bill.Button(OperatorConstants::kButtonIDTouchpad)};
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    m_swerveDrive->ResetGyroscope();
    m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}});
  },{m_swerveDrive}).ToPtr());

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
  shootButton.OnTrue(RunShooter(m_shooter, 1.0).ToPtr());
  shootButton.OnFalse(frc2::InstantCommand([this]{
    m_shooter->SetRollerPower(0.0);
  },{m_shooter}).ToPtr());

  frc2::Trigger shooterCloseRangeButton{m_ted.Button(OperatorConstants::kButtonIDX)};
  shooterCloseRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterConstants::kCloseTarget).ToPtr());

  frc2::Trigger shooterMidRangeButton{m_ted.Button(OperatorConstants::kButtonIDCircle)};
  shooterMidRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterConstants::kMidTarget).ToPtr());

  frc2::Trigger shooterFarRangeButton{m_ted.Button(OperatorConstants::kButtonIDTriangle)};
  shooterFarRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterConstants::kFarTarget).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::WaitCommand(1.0_s).ToPtr(); 
}

