// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber() : 
m_leftMotor(ClimberConstants::kLefttMotorID,rev::CANSparkMaxLowLevel::MotorType::kBrushless),
m_rightMotor(ClimberConstants::kRightMotorID,rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
m_leftServo(ClimberConstants::kLeftServoID),
m_rightServo(ClimberConstants::kRightServoID),
m_brakeModeLimitSwitch(ClimberConstants::kLimitSwitchPort)
{
    m_leftMotor.RestoreFactoryDefaults();
    m_rightMotor.RestoreFactoryDefaults();
    m_leftMotor.SetInverted(ClimberConstants::kInvertLeftMotor);
    m_rightMotor.SetInverted(ClimberConstants::kInvertRightMotor);
    m_leftMotor.SetIdleMode(ClimberConstants::kIdleMode);
    m_rightMotor.SetIdleMode(ClimberConstants::kIdleMode);
    m_leftMotor.Follow(m_rightMotor);
    // std::cout << "Climber constructing\n";
}
// This method will be called once per scheduler run
void Climber::Periodic() {
    auto limitSwitchTripped = !m_brakeModeLimitSwitch.Get();
    if (limitSwitchTripped && !m_lastLimitSwitchDetectionState) {
        ToggleGlobalBrakeMode();
    }
    m_lastLimitSwitchDetectionState = limitSwitchTripped;
}

void Climber::SetPower(double power){
    m_rightMotor.Set(power);
}

void Climber::SetServoRotation(double angle){
    m_leftServo.SetAngle(angle);
    m_rightServo.SetAngle(angle);
}

void Climber::SetBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_leftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_rightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            break;
        case(BrakeMode::Coast) :
            m_leftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_rightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            break;
        case(BrakeMode::Default) :
            m_leftMotor.SetIdleMode(ClimberConstants::kIdleMode);
            m_rightMotor.SetIdleMode(ClimberConstants::kIdleMode);
            break;
        default :
            break;
    }
}

void Climber::ToggleGlobalBrakeMode() {
    if (m_brakeMode == BrakeMode::Default) {
        m_brakeMode = BrakeMode::Coast;
    } else {
        m_brakeMode = BrakeMode::Default;
    }
}

BrakeMode Climber::GetBrakeMode() {
    return m_brakeMode;
}