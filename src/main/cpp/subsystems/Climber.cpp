// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber() : 
m_leftMotor(ClimberConstants::klefttMotorID,rev::CANSparkMaxLowLevel::MotorType::kBrushless),
m_rightMotor(ClimberConstants::krightMotorID,rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
m_leftServo(ClimberConstants::kleftServoID),
m_rightServo(ClimberConstants::krightServoID)
{
    m_leftMotor.RestoreFactoryDefaults();
    m_rightMotor.RestoreFactoryDefaults();
    m_leftMotor.SetInverted(ClimberConstants::kInvertMotor);
    m_leftMotor.Follow(m_rightMotor);

}
// This method will be called once per scheduler run
void Climber::Periodic() {}

void Climber::SetPower(double power){
    m_leftMotor.Set(power);
}

void Climber::SetServoRotation(double angle){
    m_leftServo.SetAngle(angle);
    m_rightervo.SetAngle(angle);
}
