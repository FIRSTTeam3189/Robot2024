// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include "units/current.h"
Shooter::Shooter() : 
m_shooterMotor(ShooterConstants::kMotorID, rev::CANSparkFlex::MotorType::kBrushless){

}

// This method will be called once per scheduler run
void Shooter::Periodic() {

}

void Shooter::SetPower(double shooterPower){
    m_shooterMotor.Set(shooterPower);
}
