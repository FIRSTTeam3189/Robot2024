// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber() : 
m_leftMotor(ClimberConstants::kLeftMotorID, rev::spark::SparkMax::MotorType::kBrushless),
m_rightMotor(ClimberConstants::kRightMotorID, rev::spark::SparkMax::MotorType::kBrushless), 
m_leftEncoder(m_leftMotor.GetEncoder()),
m_rightEncoder(m_rightMotor.GetEncoder()), 
m_climberConfig(),
m_brakeModeLimitSwitch(ClimberConstants::kLimitSwitchPort)
{
    ConfigMotors();
}

// This method will be called once per scheduler run
void Climber::Periodic() {
    auto limitSwitchTripped = !m_brakeModeLimitSwitch.Get();
    if (limitSwitchTripped && !m_lastLimitSwitchDetectionState) {
        ToggleGlobalBrakeMode();
    }
    m_lastLimitSwitchDetectionState = limitSwitchTripped;

    frc::SmartDashboard::PutNumber("Climber left position rots", m_leftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Climber right position rots", m_rightEncoder.GetPosition());
}

void Climber::ConfigMotors() {
    m_climberConfig
        .Inverted(ClimberConstants::kInvertRightMotor)
        .SetIdleMode(ClimberConstants::kIdleMode)
        .SmartCurrentLimit(ClimberConstants::kCurrentLimit);
    m_climberConfig.softLimit
        .ForwardSoftLimitEnabled(ClimberConstants::kSoftLimitForwardEnabled)
        .ReverseSoftLimitEnabled(ClimberConstants::kSoftLimitReverseEnabled)
        .ForwardSoftLimit(ClimberConstants::kSoftLimitForwardValue)
        .ReverseSoftLimit(ClimberConstants::kRightMotorSoftLimitReverseValue);

    m_leftMotor.Configure(m_climberConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_rightMotor.Configure(m_climberConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    // m_leftMotor.RestoreFactoryDefaults();
    // m_rightMotor.RestoreFactoryDefaults();
    // m_leftMotor.SetInverted(ClimberConstants::kInvertLeftMotor);
    // m_rightMotor.SetInverted(ClimberConstants::kInvertRightMotor);
    // m_leftMotor.SetIdleMode(ClimberConstants::kIdleMode);
    // m_rightMotor.SetIdleMode(ClimberConstants::kIdleMode);
    // m_leftMotor.SetSmartCurrentLimit(ClimberConstants::kCurrentLimit);
    // m_rightMotor.SetSmartCurrentLimit(ClimberConstants::kCurrentLimit);
    // m_leftMotor.EnableSoftLimit(ClimberConstants::kLeftMotorSoftLimitDirection, ClimberConstants::kLeftMotorSoftLimitEnabled);
    // m_rightMotor.EnableSoftLimit(ClimberConstants::kRightMotorSoftLimitDirection, ClimberConstants::kRightMotorSoftLimitEnabled);
    // m_leftMotor.SetSoftLimit(ClimberConstants::kLeftMotorSoftLimitDirection, ClimberConstants::kLeftMotorSoftLimitValue);
    // m_rightMotor.SetSoftLimit(ClimberConstants::kRightMotorSoftLimitDirection, ClimberConstants::kRightMotorSoftLimitValue);
}

void Climber::SetPower(double leftPower, double rightPower){
    m_leftMotor.Set(leftPower);
    m_rightMotor.Set(rightPower);

    frc::SmartDashboard::PutNumber("Climber left output", leftPower);
    frc::SmartDashboard::PutNumber("Climber right output", rightPower);

    frc::SmartDashboard::PutNumber("Right Motor Current (amps)", m_rightMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Left Motor Current (amps)", m_leftMotor.GetOutputCurrent());
}

void Climber::SetBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_climberConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
            break;
        case(BrakeMode::Coast) :
            m_climberConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kCoast);
            break;
        case(BrakeMode::Default) :
            m_climberConfig.SetIdleMode(ClimberConstants::kIdleMode);
            break;
        default :
            break;
    }

    m_leftMotor.Configure(m_climberConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_rightMotor.Configure(m_climberConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
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