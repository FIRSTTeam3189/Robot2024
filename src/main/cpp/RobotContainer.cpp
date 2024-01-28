// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "frc/DataLogManager.h"


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // Runs shooter and shooter angle motor continuously
  m_shooter->SetDefaultCommand(frc2::RunCommand([this] { 
    m_shooter->SetRollerPower(m_ted.GetRawAxis(OperatorConstants::kAxisLeftStickY)); 
    m_shooter->SetAnglePower(m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY));
  },{m_shooter}).ToPtr());


  m_swerveDrive->SetDefaultCommand(JoystickDrive(&m_bill, m_swerveDrive, m_isSpecialHeadingMode));
  
  m_intake->SetDefaultCommand(frc2::RunCommand([this] {
    m_intake->SetRotation(m_bill.GetRawAxis(OperatorConstants::kAxisRightStickY * 0.5));
  },{m_intake}).ToPtr());
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
   // Bill controls
  frc2::Trigger intakeButton{m_bill.Button(OperatorConstants::kButtonIDLeftBumper)};
  intakeButton.OnTrue(RunIntake(m_intake, 0.5, 0.0).ToPtr());
  intakeButton.OnFalse(frc2::InstantCommand([this]{
    m_intake->SetPower(0.0, 0.0);
  },{m_intake}).ToPtr());

  frc2::Trigger retractIntakeButton{m_bill.Button(OperatorConstants::kButtonIDX)};
  retractIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeConstants::kRetractPosition).ToPtr());

  frc2::Trigger ampScoreIntakeButton{m_bill.Button(OperatorConstants::kButtonIDCircle)};
  ampScoreIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeConstants::kAmpPosition).ToPtr());

  frc2::Trigger extendIntakeButton{m_bill.Button(OperatorConstants::kButtonIDTriangle)};
  extendIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeConstants::kExtendPosition).ToPtr());

  frc2::Trigger resetPoseButton{m_bill.Button(OperatorConstants::kButtonIDTouchpad)};
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    m_swerveDrive->ResetGyroscope();
    m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}});
  },{m_swerveDrive}).ToPtr());

  // Ted controls
  frc2::Trigger shootButton{m_ted.Button(OperatorConstants::kButtonIDRightTrigger)};
  shootButton.OnTrue(RunShooter(m_shooter, 1.0).ToPtr());
  shootButton.OnFalse(frc2::InstantCommand([this]{
    m_shooter->SetRollerPower(0.0);
  },{m_shooter}).ToPtr());

  frc2::Trigger shooterCloseRangeButton{m_ted.Button(OperatorConstants::kButtonIDX)};
  shooterCloseRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterConstants::kClosePosition).ToPtr());

  frc2::Trigger shooterMidRangeButton{m_ted.Button(OperatorConstants::kButtonIDCircle)};
  shooterMidRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterConstants::kMidPosition).ToPtr());

  frc2::Trigger shooterFarRangeButton{m_ted.Button(OperatorConstants::kButtonIDTriangle)};
  shooterFarRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterConstants::kFarPosition).ToPtr());
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::WaitCommand(1.0_s).ToPtr(); 
}

