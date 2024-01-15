// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() : 
m_frontMotor(ShooterConstants::kFrontMotorID, rev::CANSparkFlex::MotorType::kBrushless),
m_backMotor(ShooterConstants::kBackMotorID, rev::CANSparkFlex::MotorType::kBrushless) {

}

// This method will be called once per scheduler run
void Shooter::Periodic() {

}

void Shooter::SetPower(double power){
    m_frontMotor.Set(power);
    m_backMotor.Set(-power);
}
