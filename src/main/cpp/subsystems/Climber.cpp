// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber() : 
m_leftMotor(ClimberConstants::kLefttMotorID,rev::CANSparkMaxLowLevel::MotorType::kBrushless),
m_rightMotor(ClimberConstants::kRightMotorID,rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
m_leftServo(ClimberConstants::kLeftServoID),
m_rightServo(ClimberConstants::kRightServoID)
{
    m_leftMotor.RestoreFactoryDefaults();
    m_rightMotor.RestoreFactoryDefaults();
    m_leftMotor.SetInverted(ClimberConstants::kInvertLeftMotor);
    m_rightMotor.SetInverted(ClimberConstants::kInvertRightMotor);
    m_leftMotor.Follow(m_rightMotor);
    // std::cout << "Climber constructing\n";
}
// This method will be called once per scheduler run
void Climber::Periodic() {}

void Climber::SetPower(double power){
    m_rightMotor.Set(power);
}

void Climber::SetServoRotation(double angle){
    m_leftServo.SetAngle(angle);
    m_rightServo.SetAngle(angle);
}
