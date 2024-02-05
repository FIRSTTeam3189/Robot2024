// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() : 
m_rollerMotor(ShooterConstants::kRollerMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_loaderMotor(ShooterConstants::kLoaderMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_extensionMotor(ShooterConstants::kExtensionMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_rotationMotor(ShooterConstants::kRotationMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_constraints(IntakeConstants::kMaxRotationVelocity, IntakeConstants::kMaxRotationAcceleration),
m_rotationPIDController(ShooterConstants::kPRotation, ShooterConstants::kIRotation, ShooterConstants::kDRotation, m_constraints),
m_ff(ShooterConstants::kSRotation, ShooterConstants::kGRotation, ShooterConstants::kVRotation, ShooterConstants::kARotation),
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
    frc::SmartDashboard::PutNumber("Shooter PID target", m_target.value());
    frc::SmartDashboard::PutNumber("Shooter rotation", m_rotationEncoder.GetPosition());
    UpdateUltrasonic();
}

void Shooter::SetRotation(units::degree_t target) {
    // Calculates PID value in volts based on position and target
    units::volt_t PIDValue = units::volt_t{m_rotationPIDController.Calculate(GetRotation(), target)};

    // Calculates the change in velocity (acceleration) since last control loop
    // Uses the acceleration value and desired velocity to calculate feedforward gains
    // Feedforward gains are approximated based on the current state of the system and a known physics model
    // Gains calculated with SysID                                   
    auto acceleration = (m_rotationPIDController.GetSetpoint().velocity - m_lastSpeed) /
      (frc::Timer::GetFPGATimestamp() - m_lastTime);
    units::volt_t ffValue = m_ff.Calculate(units::radian_t{target}, units::radians_per_second_t{m_rotationPIDController.GetSetpoint().velocity},
                                           units::radians_per_second_squared_t{acceleration});

    // Set motor to combined voltage
    m_rotationMotor.SetVoltage(PIDValue + ffValue);

    m_lastSpeed = m_rotationPIDController.GetSetpoint().velocity;
    m_lastTime = frc::Timer::GetFPGATimestamp();
    m_target = target;
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

units::degree_t Shooter::GetRotation() {
    return units::degree_t{m_rotationEncoder.GetPosition()};
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
    m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rotationMotor.SetSmartCurrentLimit(ShooterConstants::kRotationCurrentLimit);
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

void Shooter::StopRotation() {
    m_rotationMotor.StopMotor();
}