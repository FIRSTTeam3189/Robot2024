// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() : 
m_spinMotor(ShooterConstants::kSpinMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_extensionMotor(ShooterConstants::kExtensionMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_extensionPIDController(m_extensionMotor.GetPIDController()),
m_extensionEncoder(m_extensionMotor.GetAlternateEncoder(rev::SparkMaxAlternateEncoder::AlternateEncoderType::kQuadrature, ShooterConstants::kEncoderCountsPerRev)) {
    m_extensionPIDController.SetP(ShooterConstants::kPExtension); 
    m_extensionPIDController.SetI(ShooterConstants::kIExtension); 
    m_extensionPIDController.SetD(ShooterConstants::kDExtension); 
}

// This method will be called once per scheduler run
void Shooter::Periodic() {

}

void Shooter::SetExtension(double position){
    m_extensionPIDController.SetReference(position, rev::ControlType::kPosition);
}

void Shooter::SetShootPower(double power){
    m_spinMotor.Set(power);
}

void Shooter::SetAnglePower(double power){
    m_extensionMotor.Set(power);
}
