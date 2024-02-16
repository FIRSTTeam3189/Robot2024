// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() : 
 m_rotationMotor(IntakeConstants::kRotationMotorID, rev::CANSparkMax::MotorType::kBrushless),
 m_rollerMotor(IntakeConstants::kRollerMotorID, rev::CANSparkMax::MotorType::kBrushless),
 m_constraints(IntakeConstants::kMaxRotationVelocity, IntakeConstants::kMaxRotationAcceleration),
 m_rotationPIDController(IntakeConstants::kPRotation, IntakeConstants::kIRotation, IntakeConstants::kDRotation, m_constraints),
 m_ff(IntakeConstants::kSRotation, IntakeConstants::kGRotation, IntakeConstants::kVRotation, IntakeConstants::kARotation),
 m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
 m_ultrasonicSensor(IntakeConstants::kUltrasonicPort, IntakeConstants::kUltrasonicValueRange),
 m_noteDetected(false),
 m_sysIdRoutine(
    // Might want to reduce voltage values later
    frc2::sysid::Config(std::nullopt, std::nullopt, std::nullopt, std::nullopt),
    frc2::sysid::Mechanism(
        [this](units::volt_t driveVoltage) {
          m_rotationMotor.SetVoltage(driveVoltage);
        },
        [this](frc::sysid::SysIdRoutineLog* log) {
          log->Motor("rotation")
              .voltage(m_rotationMotor.Get() *
                       frc::RobotController::GetBatteryVoltage())
              .position(units::turn_t{GetRotation()})
              .velocity(units::turns_per_second_t{units::degrees_per_second_t{m_rotationEncoder.GetVelocity()}});
        },
        this)
) 
{
    m_rotationMotor.RestoreFactoryDefaults();
    m_rollerMotor.RestoreFactoryDefaults();
    m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rotationMotor.SetSmartCurrentLimit(IntakeConstants::kRotationCurrentLimit);
    m_rotationEncoder.SetInverted(IntakeConstants::kRotationInverted);
    m_rotationEncoder.SetPositionConversionFactor(IntakeConstants::kRotationConversion);
    m_rotationEncoder.SetVelocityConversionFactor(IntakeConstants::kRotationConversion);
    m_rotationEncoder.SetZeroOffset(IntakeConstants::kRotationOffset);
    m_rollerMotor.SetInverted(IntakeConstants::kRollerInverted);

    m_rotationPKey = "Intake Rotation P";
    m_rotationIKey = "Intake Rotation I";
    m_rotationDKey = "Intake Rotation D";

    frc::Preferences::InitDouble(m_rotationPKey, IntakeConstants::kPRotation);
    frc::Preferences::InitDouble(m_rotationIKey, IntakeConstants::kIRotation);
    frc::Preferences::InitDouble(m_rotationDKey, IntakeConstants::kDRotation);

    std::cout << "Intake constructing\n";
}

// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake PID target", m_target.value());
    frc::SmartDashboard::PutNumber("Intake position", m_rotationEncoder.GetPosition());
    UpdateUltrasonic();
    if (frc::Preferences::GetBoolean("Tuning Mode?", false))
        UpdatePreferences();
}

void Intake::SetRotation(units::degree_t target) {
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
    frc::SmartDashboard::PutNumber("Intake rotation volts", PIDValue.value() + ffValue.value());

    m_lastSpeed = m_rotationPIDController.GetSetpoint().velocity;
    m_lastTime = frc::Timer::GetFPGATimestamp();
    m_target = target;
}

void Intake::SetRollerPower(double power) {
    m_rollerMotor.Set(power);
}

void Intake::SetRotationPower(double power) {
    m_rotationMotor.Set(power);
}

units::degree_t Intake::GetRotation() {
    return units::degree_t{m_rotationEncoder.GetPosition()};
}

void Intake::UpdatePreferences() {
    m_rotationPIDController.SetP(frc::Preferences::GetDouble(m_rotationPKey, IntakeConstants::kPRotation));
    m_rotationPIDController.SetI(frc::Preferences::GetDouble(m_rotationIKey, IntakeConstants::kIRotation));
    m_rotationPIDController.SetD(frc::Preferences::GetDouble(m_rotationDKey, IntakeConstants::kDRotation));
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

frc2::CommandPtr Intake::SysIdQuasistatic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Intake::SysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}