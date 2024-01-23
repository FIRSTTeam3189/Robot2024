// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() : 
 m_extensionMotor(IntakeConstants::kExtensionMotorID, rev::CANSparkFlex::MotorType::kBrushless),
 m_rollerMotor(IntakeConstants::kRollerMotorID, rev::CANSparkFlex::MotorType::kBrushless),
 m_extensionPIDController(m_extensionMotor.GetPIDController()),
 m_extensionEncoder(m_extensionMotor.GetAlternateEncoder(rev::SparkMaxAlternateEncoder::AlternateEncoderType::kQuadrature, IntakeConstants::kEncoderCountsPerRev))
{
    m_extensionPIDController.SetP(IntakeConstants::kPExtension);
    m_extensionPIDController.SetI(IntakeConstants::kIExtension);
    m_extensionPIDController.SetD(IntakeConstants::kDExtension);
    m_extensionPIDController.SetFeedbackDevice(m_extensionEncoder);
}

// This method will be called once per scheduler run
void Intake::Periodic() {}

void Intake::SetExtension(double position) {
    m_extensionPIDController.SetReference(position, rev::ControlType::kPosition);
}


void Intake::SetPower(double rollerPower, double extensionPower) {
    m_rollerMotor.Set(rollerPower);
    m_extensionMotor.Set(extensionPower);
}