// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() : 
m_rollerMotor(ShooterConstants::kRollerMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_loaderMotor(ShooterConstants::kLoaderMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_extensionMotor(ShooterConstants::kExtensionMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_rotationMotor(ShooterConstants::kRotationMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_rotationPIDController(m_rotationMotor.GetPIDController()),
m_extensionPIDController(m_extensionMotor.GetPIDController()),
m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)), 
m_extensionEncoder(m_extensionMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
m_ultrasonicSensor(ShooterConstants::kUltrasonicPort, ShooterConstants::kUltrasonicValueRange),
m_noteDetected(false) {
    ConfigRollerMotor();
    ConfigExtensionMotor();
    ConfigRotationMotor();
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
    UpdateUltrasonic();
}

void Shooter::SetRotation(double target) {
    m_rotationPIDController.SetReference(target, rev::ControlType::kPosition);
}

void Shooter::SetExtension(double target) {
    m_extensionPIDController.SetReference(target, rev::ControlType::kPosition);
}

void Shooter::SetRollerPower(double power) {
    m_rollerMotor.Set(power);
}

void Shooter::SetRotationPower(double power) {
    m_rotationMotor.Set(power);
}

void Shooter::SetExtensionPower(double power) {
    m_extensionMotor.Set(power);
}

void Shooter::SetLoaderPower(double power) {
    m_loaderMotor.Set(power);
}

double Shooter::GetRotation() {
    return m_rotationEncoder.GetPosition();
}

double Shooter::GetExtension() {
    return m_extensionEncoder.GetPosition();
}


void Shooter::ConfigRollerMotor() {
    m_rollerMotor.RestoreFactoryDefaults();
}

void Shooter::ConfigExtensionMotor() {
    m_extensionMotor.RestoreFactoryDefaults();
    m_extensionPIDController.SetFeedbackDevice(m_extensionEncoder);
    m_extensionPIDController.SetP(ShooterConstants::kPExtension);
    m_extensionPIDController.SetI(ShooterConstants::kIExtension);
    m_extensionPIDController.SetD(ShooterConstants::kDExtension);
    m_extensionEncoder.SetInverted(ShooterConstants::kExtensionInverted);
    m_extensionEncoder.SetPositionConversionFactor(ShooterConstants::kExtensionConversion);
    m_extensionEncoder.SetZeroOffset(ShooterConstants::kExtensionOffset);
}

void Shooter::ConfigRotationMotor() {
    m_rotationMotor.RestoreFactoryDefaults();
    m_rotationPIDController.SetFeedbackDevice(m_extensionEncoder);
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