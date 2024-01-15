// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() : 
 m_extensionMotor(IntakeConstants::kExtensionMotorID, rev::CANSparkFlex::MotorType::kBrushless),
 m_rollerMotor(IntakeConstants::kRollerMotorID, rev::CANSparkMax::MotorType::kBrushless)
{

}

// This method will be called once per scheduler run
void Intake::Periodic() {}

void Intake::SetPower(double rollerPower, double extensionPower) {
    m_rollerMotor.Set(rollerPower);
    m_extensionMotor.Set(extensionPower);
}