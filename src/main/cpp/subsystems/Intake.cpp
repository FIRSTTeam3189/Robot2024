// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() : 
 m_rotationMotor(IntakeConstants::kRotationMotorID, rev::CANSparkMax::MotorType::kBrushless),
 m_rollerMotor(IntakeConstants::kRollerMotorID, rev::CANSparkMax::MotorType::kBrushless),
 m_constraints(IntakeConstants::kMaxRotationVelocity, IntakeConstants::kMaxRotationAcceleration),
 m_profiledPIDController(IntakeConstants::kPRotation, IntakeConstants::kIRotation, IntakeConstants::kDRotation, m_constraints),
 m_rotationPIDController(m_rotationMotor.GetPIDController()),
 m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
 m_target(IntakeConstants::kRetractTarget),
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
),
m_currentState(IntakeState::Retracted),
m_prevState(IntakeState::None)
{
    ConfigRotationMotor();
    ConfigRollerMotor();
    ConfigPID();

    std::cout << "Intake constructed\n";
}

void Intake::ConfigRotationMotor() {
    m_rotationMotor.RestoreFactoryDefaults();
    m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rotationMotor.SetSmartCurrentLimit(IntakeConstants::kRotationCurrentLimit);
    m_rotationMotor.SetInverted(IntakeConstants::kRotationMotorInverted);
    m_rotationEncoder.SetInverted(IntakeConstants::kRotationInverted);
    m_rotationEncoder.SetPositionConversionFactor(IntakeConstants::kRotationConversion);
    m_rotationEncoder.SetVelocityConversionFactor(IntakeConstants::kRotationConversion);
    m_rotationEncoder.SetZeroOffset(IntakeConstants::kRotationOffset);
}

void Intake::ConfigRollerMotor() {
    m_rollerMotor.RestoreFactoryDefaults();
    m_rollerMotor.SetSmartCurrentLimit(IntakeConstants::kRollerCurrentLimit);
    m_rollerMotor.SetInverted(IntakeConstants::kRollerInverted);
}

void Intake::ConfigPID() {
    m_ff = new frc::ArmFeedforward(
        IntakeConstants::kSRotation,
        IntakeConstants::kGRotation,
        IntakeConstants::kVRotation,
        IntakeConstants::kARotation
    );

    m_rotationPKey = "Intake Rotation P";
    m_rotationIKey = "Intake Rotation I";
    m_rotationDKey = "Intake Rotation D";
    m_rotationGKey = "Intake Rotation G";
    m_rotationSKey = "Intake Rotation S";
    m_rotationVKey = "Intake Rotation V";
    m_rotationAKey = "Intake Rotation A";

    frc::Preferences::InitDouble(m_rotationPKey, IntakeConstants::kPRotation);
    frc::Preferences::InitDouble(m_rotationIKey, IntakeConstants::kIRotation);
    frc::Preferences::InitDouble(m_rotationDKey, IntakeConstants::kDRotation);
    frc::Preferences::InitDouble(m_rotationGKey, IntakeConstants::kGRotation.value());
    frc::Preferences::InitDouble(m_rotationSKey, IntakeConstants::kSRotation.value());
    frc::Preferences::InitDouble(m_rotationVKey, IntakeConstants::kVRotation.value());
    frc::Preferences::InitDouble(m_rotationAKey, IntakeConstants::kARotation.value());

    m_rotationPIDController.SetFeedbackDevice(m_rotationEncoder);
    m_rotationPIDController.SetP(kRotationTargetPID[IntakeState::Extended][0]);
    m_rotationPIDController.SetI(kRotationTargetPID[IntakeState::Extended][1]);
    m_rotationPIDController.SetD(kRotationTargetPID[IntakeState::Extended][2]);

    m_rotationPIDController.SetP(kRotationTargetPID[IntakeState::Amp][0], 1);
    m_rotationPIDController.SetI(kRotationTargetPID[IntakeState::Amp][1], 1);
    m_rotationPIDController.SetD(kRotationTargetPID[IntakeState::Amp][2], 1);

    m_rotationPIDController.SetP(kRotationTargetPID[IntakeState::Retracted][0], 2);
    m_rotationPIDController.SetI(kRotationTargetPID[IntakeState::Retracted][1], 2);
    m_rotationPIDController.SetD(kRotationTargetPID[IntakeState::Retracted][2], 2);
}

// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake PID target", m_target.value());
    frc::SmartDashboard::PutNumber("Intake rotation", m_rotationEncoder.GetPosition());
    UpdateUltrasonic();
    if (frc::Preferences::GetBoolean("Tuning Mode?", false))
        UpdatePreferences();
    SetRotation(m_target);
}

units::degree_t Intake::GetTarget() {
    return m_target;
}

void Intake::SetState(IntakeState state) {
    auto target = 0.0_deg;
    m_target = target;

    switch(state){
        case (IntakeState::None) :
            break;
        case (IntakeState::Extended) :
            target = IntakeConstants::kExtendTarget;
            break;
        case (IntakeState::Amp) :
            target = IntakeConstants::kAmpTarget;
            break;
        case (IntakeState::Retracted) :
            target = IntakeConstants::kRetractTarget;
            break;
    }

    if (target >= 180.0_deg){
        target -= 360.0_deg;
    }

    SetRotation(target);
}

void Intake::SetRotation(units::degree_t target) {
    // Calculates PID value in volts based on position and target
    units::volt_t PIDValue = units::volt_t{m_profiledPIDController.Calculate(GetRotation(), target)};

    // Calculates the change in velocity (acceleration) since last control loop
    // Uses the acceleration value and desired velocity to calculate feedforward gains
    // Feedforward gains are approximated based on the current state of the system and a known physics model
    // Gains calculated with SysID                                   
    auto acceleration = (m_profiledPIDController.GetSetpoint().velocity - m_lastSpeed) /
      (frc::Timer::GetFPGATimestamp() - m_lastTime);
    units::volt_t ffValue = m_ff->Calculate(units::radian_t{target}, units::radians_per_second_t{m_profiledPIDController.GetSetpoint().velocity},
                                           units::radians_per_second_squared_t{acceleration});

    // Set motor to combined voltage
    if (GetRotation() <= 15.0_deg && (PIDValue + ffValue).value() <= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else if (GetRotation() >= 110.0_deg && (PIDValue + ffValue).value() >= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else 
        m_rotationMotor.SetVoltage(PIDValue + ffValue);

    // switch (target) {
    // case IntakeConstants::kRetractTarget :
    //     m_prevState = m_currentState;
    //     m_currentState = IntakeState::Retracted;
    //     break;

    // case IntakeConstants::kExtendTarget :
    //     m_prevState = m_currentState;
    //     m_currentState = IntakeState::Extended;
    //     break;

    // case IntakeConstants::kAmpTarget :
    //     m_prevState = m_currentState;
    //     m_currentState = IntakeState::Amp;
    //     break;
    
    // default:
    //     break;
    // }

    // double ff = 0.0;
    // if (GetRotation() < 15.0_deg && target < GetRotation())
    //     m_rotationMotor.SetVoltage(0.0_V);
    // else if (GetRotation() > 110.0_deg && target > GetRotation()){
    //     m_rotationMotor.SetVoltage(0.0_V);
    // }
    // else {
        // if (target < GetRotation()) {
        //     m_rotationPIDController.SetP(IntakeConstants::kPRotation / 1.25);
        //     // ff = -IntakeConstants::kFeedforward / 2.0;
        // }
        // else {
        //     m_rotationPIDController.SetP(IntakeConstants::kPRotation);
        //     ff = IntakeConstants::kFeedforward;
        // }

        // if (m_currentState == IntakeState::Extended) {
        //     // m_rotationPIDController.SetReference(target.value(), rev::ControlType::kPosition, 0, ff);
        // } else if (m_currentState == IntakeState::Amp) {
        //     // m_rotationPIDController.SetReference(target.value(), rev::ControlType::kPosition, 1, ff);
        // } else {
        //     m_rotationPIDController.SetReference(target.value(), rev::ControlType::kPosition, 2, ff);
        // }
    // }

    frc::SmartDashboard::PutNumber("Intake rotation volts", PIDValue.value() + ffValue.value());

    m_lastSpeed = m_profiledPIDController.GetSetpoint().velocity;
    m_lastTime = frc::Timer::GetFPGATimestamp();
}

void Intake::SetRollerPower(double power) {
    m_rollerMotor.Set(power);
}

void Intake::SetRotationPower(double power) {
    if (GetRotation() <= 15.0_deg && power >= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else if (GetRotation() >= 120.0_deg && power <= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else
        m_rotationMotor.Set(power);
}

units::degree_t Intake::GetRotation() {
    return units::degree_t{m_rotationEncoder.GetPosition()};
}

void Intake::UpdatePreferences() {
    m_profiledPIDController.SetP(frc::Preferences::GetDouble(m_rotationPKey, IntakeConstants::kPRotation));
    m_profiledPIDController.SetI(frc::Preferences::GetDouble(m_rotationIKey, IntakeConstants::kIRotation));
    m_profiledPIDController.SetD(frc::Preferences::GetDouble(m_rotationDKey, IntakeConstants::kDRotation));
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

void Intake::UpdateUltrasonic() {
    if (m_ultrasonicSensor.Get() < 12.0)
        m_noteDetected = true;
    else
        m_noteDetected = false;

    frc::SmartDashboard::PutNumber("Intake ultrasonic", m_ultrasonicSensor.Get());
}

bool Intake::NoteDetected() {
    // return m_noteDetected;
    return false;
}

frc2::CommandPtr Intake::SysIdQuasistatic(frc2::sysid::Direction direction) {
    for (int i = 0; i < 10; i++) 
        std::cout << "Intake Quasistatic\n";
    return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Intake::SysIdDynamic(frc2::sysid::Direction direction) {
    for (int i = 0; i < 10; i++) 
        std::cout << "Intake Dynamic\n";
    return m_sysIdRoutine.Dynamic(direction);
}