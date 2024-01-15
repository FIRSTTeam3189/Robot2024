// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <commands/RunShooter.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_shooter->SetDefaultCommand(frc2::RunCommand([this]{ 
    m_shooter->SetPower(m_ted.GetRawAxis(OperatorConstants::kAxisLeftStickY)); 
  },{m_shooter}).ToPtr());

  m_swerveDrive->SetDefaultCommand(JoystickDrive(&m_bill, m_swerveDrive, m_isSpecialHeadingMode));
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  frc2::Trigger intakeButton{m_bill.Button(OperatorConstants::kButtonIDLeftBumper)};
  intakeButton.OnTrue(RunIntake(m_intake, 1.0, 0.0).ToPtr());
  intakeButton.OnFalse(frc2::InstantCommand([this]{
    m_intake->SetPower(0.0, 0.0);
  },{m_intake}).ToPtr());

  frc2::Trigger shootButton{m_ted.Button(OperatorConstants::kButtonIDRightTrigger)};
  shootButton.OnTrue(RunShooter(m_shooter, 1.0).ToPtr());
  shootButton.OnFalse(frc2::InstantCommand([this]{
    m_shooter->SetPower(0.0);
  },{m_shooter}).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::WaitCommand(1.0_s).ToPtr();
}