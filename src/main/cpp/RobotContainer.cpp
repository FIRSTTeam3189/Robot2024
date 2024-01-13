// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_shooter->SetDefaultCommand(frc2::RunCommand([this]{ 
    m_shooter->SetPower(m_ted.GetRawAxis(OperatorConstants::kAxisLStickY)); 
  },{m_shooter}).ToPtr());
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::WaitCommand(1.0_s).ToPtr();
}
