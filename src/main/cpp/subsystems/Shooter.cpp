// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() : 
m_topMotor(ShooterConstants::kTopMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_bottomMotor(ShooterConstants::kBottomMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_loaderMotor(ShooterConstants::kLoaderMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_leftExtensionMotor(ShooterConstants::kLeftExtensionMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_rightExtensionMotor(ShooterConstants::kRightExtensionMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_rotationMotor(ShooterConstants::kRotationMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_rotationPIDController(m_rotationMotor.GetPIDController()),
m_leftExtensionPIDController(m_leftExtensionMotor.GetPIDController()),
m_rightExtensionPIDController(m_rightExtensionMotor.GetPIDController()),
m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)), 
m_leftExtensionEncoder(m_leftExtensionMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)), 
m_rightExtensionEncoder(m_rightExtensionMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
m_ultrasonicSensor(ShooterConstants::kUltrasonicPort, ShooterConstants::kUltrasonicValueRange),
m_noteDetected(false) {
    ConfigRollerMotor();
    ConfigExtensionMotors();
    ConfigRotationMotor();
   
}

// This method will be called once per scheduler run
void Shooter::Periodic() {

}

void Shooter::SetRotation(double position){
    m_rotationPIDController.SetReference(position, rev::ControlType::kPosition);
}

void Shooter::SetRollerPower(double power) {
    m_topMotor.Set(power);
}

void Shooter::SetAnglePower(double power){
    m_rotationMotor.Set(power);
}

void Shooter::SetLoaderPower(double power) {
    m_loaderMotor.Set(power);
}

void Shooter::ConfigRollerMotor() {
    m_topMotor.RestoreFactoryDefaults();
    m_topMotor.Follow(m_bottomMotor);
}

void Shooter::ConfigExtensionMotors() {
    m_leftExtensionMotor.RestoreFactoryDefaults();
    m_rightExtensionMotor.RestoreFactoryDefaults();
    m_leftExtensionPIDController.SetFeedbackDevice(m_leftExtensionEncoder);
    m_rightExtensionPIDController.SetFeedbackDevice(m_rightExtensionEncoder);
    m_leftExtensionPIDController.SetP(ShooterConstants::kPExtension);
    m_leftExtensionPIDController.SetI(ShooterConstants::kIExtension);
    m_leftExtensionPIDController.SetI(ShooterConstants::kDExtension);
    m_rightExtensionPIDController.SetP(ShooterConstants::kPExtension);
    m_rightExtensionPIDController.SetI(ShooterConstants::kIExtension);
    m_rightExtensionPIDController.SetD(ShooterConstants::kDExtension);
}

void Shooter::ConfigRotationMotor() {
    m_rotationMotor.RestoreFactoryDefaults();
    m_rotationPIDController.SetFeedbackDevice(m_leftExtensionEncoder);
    m_rotationPIDController.SetFeedbackDevice(m_rightExtensionEncoder);
    m_rotationMotor.SetSmartCurrentLimit(ShooterConstants::kRotationCurrentLimit);
    m_rotationPIDController.SetP(ShooterConstants::kPRotation); 
    m_rotationPIDController.SetI(ShooterConstants::kIRotation); 
    m_rotationPIDController.SetD(ShooterConstants::kDRotation); 
    m_rotationEncoder.SetInverted(ShooterConstants::kRotationInverted);
    m_rotationEncoder.SetPositionConversionFactor(ShooterConstants::kRotationConversion);
    m_rotationEncoder.SetZeroOffset(ShooterConstants::kRotationOffset);
}

void Shooter::UpdateUltrasonic() {
    if (m_ultrasonicSensor.Get() < 12.0)
        m_noteDetected = true;
    else
        m_noteDetected = false;

    frc::SmartDashboard::PutNumber("Shooter ultrasonic", m_ultrasonicSensor.Get());
}

bool Shooter::NoteDetected() {
    return m_noteDetected;
}