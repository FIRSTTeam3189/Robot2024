// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() : 
 m_rotationMotor(IntakeConstants::kRotationMotorID, rev::CANSparkFlex::MotorType::kBrushless),
 m_rollerMotor(IntakeConstants::kRollerMotorID, rev::CANSparkFlex::MotorType::kBrushless),
 m_rotationPIDController(m_rotationMotor.GetPIDController()),
 m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
 m_ultrasonicSensor(IntakeConstants::kUltrasonicPort, IntakeConstants::kUltrasonicValueRange),
 m_noteDetected(false)
{
    m_rotationMotor.RestoreFactoryDefaults();
    m_rollerMotor.RestoreFactoryDefaults();
    m_rotationMotor.SetSmartCurrentLimit(IntakeConstants::kRotationCurrentLimit);
    m_rotationPIDController.SetP(IntakeConstants::kPRotation);
    m_rotationPIDController.SetI(IntakeConstants::kIRotation);
    m_rotationPIDController.SetD(IntakeConstants::kDRotation);
    m_rotationPIDController.SetFeedbackDevice(m_rotationEncoder);
    m_rotationEncoder.SetInverted(IntakeConstants::kRotationInverted);
    m_rotationEncoder.SetPositionConversionFactor(IntakeConstants::kRotationConversion);
    m_rotationEncoder.SetZeroOffset(IntakeConstants::kRotationOffset);
}

// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake PID target", m_target);
    frc::SmartDashboard::PutNumber("Intake position", m_rotationEncoder.GetPosition());
    UpdateUltrasonic();
}

void Intake::SetRotation(double target) {
    m_target = target;
    m_rotationPIDController.SetReference(target, rev::ControlType::kPosition);
}

void Intake::SetRollerPower(double power) {
    m_rollerMotor.Set(power);
}

void Intake::SetRotationPower(double power) {
    m_rotationMotor.Set(power);
}

double Intake::GetRotation() {
    return m_rotationEncoder.GetPosition();
}

void Intake::UpdateUltrasonic() {
    if (m_ultrasonicSensor.Get() < 12.0)
        m_noteDetected = true;
    else
        m_noteDetected = false;

    frc::SmartDashboard::PutNumber("Intake ultrasonic", m_ultrasonicSensor.Get());
}

bool Intake::NoteDetected() {
    return m_noteDetected;
}