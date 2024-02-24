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
// m_rotationPIDController(m_rotationMotor.GetPIDController()),
m_extensionPIDController(m_extensionMotor.GetPIDController()),
m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)), 
m_extensionEncoder(m_extensionMotor.GetAlternateEncoder(rev::SparkMaxAlternateEncoder::Type::kQuadrature, ShooterConstants::kExtensionCountsPerRev)),
m_ultrasonicSensor(ShooterConstants::kUltrasonicPort, ShooterConstants::kUltrasonicValueRange),
m_noteDetected(false),
m_target(ShooterConstants::kRetractTarget),
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
) {
    
    ConfigRollerMotor();
    ConfigExtensionMotor();
    ConfigRotationMotor();
    ConfigPID();

    std::cout << "Shooter constructing\n";
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
    frc::SmartDashboard::PutNumber("Shooter PID target", m_target.value());
    frc::SmartDashboard::PutNumber("Shooter rotation", m_rotationEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Shooter extension", m_extensionEncoder.GetPosition());
    UpdateUltrasonic();
    if (frc::Preferences::GetBoolean("Tuning Mode?", false))
        UpdatePreferences();
    SetRotation(m_target);
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
    units::volt_t ffValue = m_ff->Calculate(units::radian_t{target}, units::radians_per_second_t{m_rotationPIDController.GetSetpoint().velocity},
                                           units::radians_per_second_squared_t{acceleration});

    // Set motor to combined voltage
    m_rotationMotor.SetVoltage(PIDValue + ffValue);
    frc::SmartDashboard::PutNumber("Shooter rotation volts", PIDValue.value() + ffValue.value());

    m_lastSpeed = m_rotationPIDController.GetSetpoint().velocity;
    m_lastTime = frc::Timer::GetFPGATimestamp();

    // double ff = 0.0;
     if (GetRotation() < 10.0_deg && (PIDValue + ffValue).value() <= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else if (GetRotation() > 60.0_deg && (PIDValue + ffValue).value() >= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else {
        m_rotationMotor.SetVoltage((PIDValue + ffValue));
        // if (target < GetRotation()) {
        //     m_rotationPIDController.SetP(ShooterConstants::kPRotation / 1.25);
        //     // ff = ShooterConstants::kFeedforward / 2.0;
        // }
        // else {
        //     m_rotationPIDController.SetP(ShooterConstants::kPRotation);
        //     ff = ShooterConstants::kFeedforward;
        // }
        // m_rotationPIDController.SetReference(target.value(), rev::ControlType::kPosition, 0, ff);
    }

}

void Shooter::SetExtension(double target) {
    m_extensionPIDController.SetReference(target, rev::ControlType::kPosition);
}

void Shooter::SetRollerPower(double power) {
    m_rollerMotor.Set(power);
    frc::SmartDashboard::PutNumber("Shooter power", m_rollerMotor.Get());
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
    m_rollerMotor.SetSmartCurrentLimit(ShooterConstants::kRollerCurrentLimit);
    m_rollerMotor.SetInverted(ShooterConstants::kRollerInverted);
}

void Shooter::ConfigExtensionMotor() {
    m_extensionMotor.RestoreFactoryDefaults();
    m_extensionMotor.SetSmartCurrentLimit(ShooterConstants::kExtensionCurrentLimit);
    m_extensionPIDController.SetFeedbackDevice(m_extensionEncoder);
    m_extensionPIDController.SetP(ShooterConstants::kPExtension);
    m_extensionPIDController.SetI(ShooterConstants::kIExtension);
    m_extensionPIDController.SetD(ShooterConstants::kDExtension);
    m_extensionEncoder.SetInverted(ShooterConstants::kExtensionInverted);
    m_extensionEncoder.SetPositionConversionFactor(ShooterConstants::kExtensionConversion);
}

void Shooter::ConfigRotationMotor() {
    m_rotationMotor.RestoreFactoryDefaults();
    m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rotationMotor.SetSmartCurrentLimit(ShooterConstants::kRotationCurrentLimit);
    m_rotationEncoder.SetInverted(ShooterConstants::kRotationInverted);
    m_rotationEncoder.SetPositionConversionFactor(ShooterConstants::kRotationConversion);
    m_rotationEncoder.SetVelocityConversionFactor(ShooterConstants::kRotationConversion);
    m_rotationEncoder.SetZeroOffset(ShooterConstants::kRotationOffset);
    // m_rotationPIDController.SetFeedbackDevice(m_rotationEncoder);
    // m_rotationPIDController.SetP(ShooterConstants::kPRotation);
    // m_rotationPIDController.SetI(ShooterConstants::kIRotation);
    // m_rotationPIDController.SetD(ShooterConstants::kDRotation);
}

void Shooter::ConfigPID() {
    m_ff = new frc::ArmFeedforward(
        IntakeConstants::kSRotation,
        IntakeConstants::kGRotation,
        IntakeConstants::kVRotation,
        IntakeConstants::kARotation
    );

    m_rotationPKey = "Shooter Rotation P";
    m_rotationIKey = "Shooter Rotation I";
    m_rotationDKey = "Shooter Rotation D";
    m_rotationGKey = "Shooter Rotation G";
    m_rotationSKey = "Shooter Rotation S";
    m_rotationVKey = "Shooter Rotation V";
    m_rotationAKey = "Shooter Rotation A";

    frc::Preferences::InitDouble(m_rotationPKey, ShooterConstants::kPRotation);
    frc::Preferences::InitDouble(m_rotationIKey, ShooterConstants::kIRotation);
    frc::Preferences::InitDouble(m_rotationDKey, ShooterConstants::kDRotation);
    frc::Preferences::InitDouble(m_rotationGKey, ShooterConstants::kGRotation.value());
    frc::Preferences::InitDouble(m_rotationSKey, ShooterConstants::kSRotation.value());
    frc::Preferences::InitDouble(m_rotationVKey, ShooterConstants::kVRotation.value());
    frc::Preferences::InitDouble(m_rotationAKey, ShooterConstants::kARotation.value());
}

void Shooter::UpdatePreferences() {
    m_rotationPIDController.SetP(frc::Preferences::GetDouble(m_rotationPKey, ShooterConstants::kPRotation));
    m_rotationPIDController.SetI(frc::Preferences::GetDouble(m_rotationIKey, ShooterConstants::kIRotation));
    m_rotationPIDController.SetD(frc::Preferences::GetDouble(m_rotationDKey, ShooterConstants::kDRotation));
    double s = frc::Preferences::GetDouble(m_rotationSKey, IntakeConstants::kSRotation.value());
    double g = frc::Preferences::GetDouble(m_rotationGKey, IntakeConstants::kGRotation.value());
    double v = frc::Preferences::GetDouble(m_rotationVKey, IntakeConstants::kVRotation.value());
    double a = frc::Preferences::GetDouble(m_rotationAKey, IntakeConstants::kARotation.value());
    delete m_ff;
    m_ff = new frc::ArmFeedforward(
        units::volt_t{s},
        units::volt_t{g},
        units::unit_t<frc::ArmFeedforward::kv_unit>{v},
        units::unit_t<frc::ArmFeedforward::ka_unit>{a}
    );
}

void Shooter::UpdateUltrasonic() {
    if (m_ultrasonicSensor.Get() < 12.0)
        m_noteDetected = true;
    else
        m_noteDetected = false;

    frc::SmartDashboard::PutNumber("Shooter ultrasonic", m_ultrasonicSensor.Get());
}

bool Shooter::NoteDetected() {
    // return m_noteDetected;
    return false;
}

frc2::CommandPtr Shooter::SysIdQuasistatic(frc2::sysid::Direction direction) {
    for (int i = 0; i < 10; i++) 
        std::cout << "Shooter Quasistatic\n";
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Shooter::SysIdDynamic(frc2::sysid::Direction direction) {
    for (int i = 0; i < 10; i++) 
        std::cout << "Shooter Quasistatic\n";
  return m_sysIdRoutine.Dynamic(direction);
}

void Shooter::SetState(ShooterState state){
    auto target = 0.0_deg;
    m_target = target;

    switch(state){
        case(ShooterState::None):
            break;
        case(ShooterState::Retracted):
            target = ShooterConstants::kRetractTarget;
            break;
        case(ShooterState::Load):
            target = ShooterConstants::kLoadTarget;
            break;
        case(ShooterState::Close):
            target = ShooterConstants::kCloseTarget;
            break;
        case(ShooterState::Mid):
            target = ShooterConstants::kMidTarget;
            break;
        case(ShooterState::Far):
            target = ShooterConstants::kFarTarget;
            break;
        default: 
            break;
    }
    SetRotation(target);
}

units::degree_t Shooter::GetTarget(){
    return m_target;
}