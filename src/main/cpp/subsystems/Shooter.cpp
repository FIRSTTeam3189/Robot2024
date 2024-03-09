// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() : 
m_rollerMotor(ShooterConstants::kRollerMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_encoder(m_rollerMotor.GetEncoder()),
m_loaderMotor(ShooterConstants::kLoaderMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_rotationMotor(ShooterConstants::kRotationMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_limitSwitchLeft(ShooterConstants::kLeftLimitSwitchPort),
m_limitSwitchRight(ShooterConstants::kRightLimitSwitchPort),
m_constraints(ShooterConstants::kMaxRotationVelocity, ShooterConstants::kMaxRotationAcceleration),
m_profiledPIDController(ShooterConstants::kPRotation, ShooterConstants::kIRotation, ShooterConstants::kDRotation, m_constraints),
m_rotationPIDController(m_rotationMotor.GetPIDController()),
m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)), 
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
),
m_isActive(false) {
    ConfigRollerMotor();
    ConfigLoaderMotor();
    ConfigRotationMotor();
    ConfigPID();

    // std::cout << "Shooter constructing\n";
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
    frc::SmartDashboard::PutNumber("Shooter PID target", m_target.value());
    frc::SmartDashboard::PutNumber("Shooter rotation", m_rotationEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Shooter power", m_rollerMotor.Get());
    frc::SmartDashboard::PutNumber("Shooter RPM", m_encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter load power", m_loaderMotor.Get());

    if (frc::Preferences::GetBoolean("Full Diagnostics", false)) {
        frc::SmartDashboard::PutNumber("Shooter rotational velocity", m_rotationEncoder.GetVelocity());
        frc::SmartDashboard::PutNumber("Shooter desired rotational velocity", m_profiledPIDController.GetSetpoint().velocity.value());
        frc::SmartDashboard::PutNumber("Shooter rotational acceleration", m_acceleration.value());
        frc::SmartDashboard::PutNumber("Shooter desired rotational acceleration", m_targetAcceleration.value());
    }

    if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
        UpdatePreferences();
    }

    if (m_isActive) {
        SetRotation(m_target);
    } else {
        HoldPosition(m_target);
    }
}

void Shooter::HoldPosition(units::degree_t target) {
    units::volt_t PIDValue = 0.0_V;
    if (fabs(m_target.value() - GetRotation().value()) > ShooterConstants::kRotationIdleTolerance.value()) {
        PIDValue = units::volt_t{(m_target - GetRotation()).value() * m_profiledPIDController.GetP()};
    }

    auto ffValue = m_ff->Calculate(units::radian_t{GetRotation()}, units::radians_per_second_t{0.0});
    frc::SmartDashboard::PutNumber("Shooter rotation FF", ffValue.value());
    frc::SmartDashboard::PutNumber("Shooter rotation PID", PIDValue.value());
    m_rotationMotor.SetVoltage(std::clamp((ffValue + PIDValue), -12.0_V, 12.0_V));
    // Just to advance the profile timestep
    m_profiledPIDController.Calculate(GetRotation(), m_target);
}

void Shooter::SetRotation(units::degree_t target) {
    // Calculates PID value in volts based on position and target
    // units::volt_t PIDValue = units::volt_t{m_profiledPIDController.Calculate(GetRotation(), target)};
    units::volt_t PIDValue = units::volt_t{(target - GetRotation()).value() * m_profiledPIDController.GetP()};
    m_profiledPIDController.Calculate(GetRotation(), m_target);

    // Calculates the change in velocity (acceleration) since last control loop
    // Uses the acceleration value and desired velocity to calculate feedforward gains
    // Feedforward gains are approximated based on the current state of the system and a known physics model
    // Gains calculated with SysID
    m_acceleration = (units::degrees_per_second_t{m_rotationEncoder.GetVelocity()} - m_lastSpeed) /
      (frc::Timer::GetFPGATimestamp() - m_lastTime);

    m_targetAcceleration = (m_profiledPIDController.GetSetpoint().velocity - m_lastTargetSpeed) /
      (frc::Timer::GetFPGATimestamp() - m_lastTime);
    units::volt_t ffValue = m_ff->Calculate(units::radian_t{target}, units::radians_per_second_t{m_profiledPIDController.GetSetpoint().velocity},
                                           units::radians_per_second_squared_t{m_targetAcceleration});

    // Set motor to combined voltage
    m_rotationMotor.SetVoltage(PIDValue + ffValue);
    frc::SmartDashboard::PutNumber("Shooter rotation volts", PIDValue.value() + ffValue.value());
    frc::SmartDashboard::PutNumber("Shooter rotation PID", PIDValue.value());
    frc::SmartDashboard::PutNumber("Shooter rotation FF", ffValue.value());

    m_lastTargetSpeed = m_profiledPIDController.GetSetpoint().velocity;
    m_lastSpeed = units::degrees_per_second_t{m_rotationEncoder.GetVelocity()};
    m_lastTime = frc::Timer::GetFPGATimestamp();

    // double ff = 0.0;
     if (GetRotation() < 10.0_deg && (PIDValue + ffValue).value() <= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else if (GetRotation() > 60.0_deg && (PIDValue + ffValue).value() >= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else {
        m_rotationMotor.SetVoltage(std::clamp((PIDValue + ffValue), -12.0_V, 12.0_V));
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
    // if (m_currentState == ShooterState::Load) {
    //         // m_rotationPIDController.SetReference(target.value(), rev::ControlType::kPosition, 0, ff);
    //     } else if (m_currentState == ShooterState::Retracted) {
    //         // m_rotationPIDController.SetReference(target.value(), rev::ControlType::kPosition, 1, ff);
    //     } else {
    //         m_rotationPIDController.SetReference(target.value(), rev::ControlType::kPosition, 2, ff);
    //     }
    // }


void Shooter::SetRollerPower(double power) {
    m_rollerMotor.Set(power);
}

void Shooter::SetRotationPower(double power) {
    m_rotationMotor.Set(power);
}

void Shooter::SetLoaderPower(double power) {
    m_loaderMotor.Set(power);
}

units::degree_t Shooter::GetRotation() {
    return units::degree_t{m_rotationEncoder.GetPosition()};
}

void Shooter::ConfigRollerMotor() {
    m_rollerMotor.RestoreFactoryDefaults();
    m_rollerMotor.SetSmartCurrentLimit(ShooterConstants::kRollerCurrentLimit);
    m_rollerMotor.SetInverted(ShooterConstants::kRollerInverted);
}

void Shooter::ConfigLoaderMotor() {
    m_loaderMotor.RestoreFactoryDefaults();
    m_loaderMotor.SetSmartCurrentLimit(ShooterConstants::kLoaderCurrentLimit);
}

void Shooter::ConfigRotationMotor() {
    m_rotationMotor.RestoreFactoryDefaults();
    m_rotationMotor.SetIdleMode(ShooterConstants::kIdleMode);
    m_rotationMotor.SetSmartCurrentLimit(ShooterConstants::kRotationCurrentLimit);
    m_rotationMotor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus5, 20);
    m_rotationMotor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus6, 20);
    m_rotationEncoder.SetInverted(ShooterConstants::kRotationInverted);
    m_rotationEncoder.SetPositionConversionFactor(ShooterConstants::kRotationConversion);
    m_rotationEncoder.SetVelocityConversionFactor(ShooterConstants::kRotationConversion);
    m_rotationEncoder.SetZeroOffset(ShooterConstants::kRotationOffset);
}

void Shooter::ConfigPID() {
    m_ff = new frc::ArmFeedforward(
        ShooterConstants::kSRotation,
        ShooterConstants::kGRotation,
        ShooterConstants::kVRotation,
        ShooterConstants::kARotation
    );
    m_rotationPKey = "Shooter Rotation P";
    m_rotationIKey = "Shooter Rotation I";
    m_rotationDKey = "Shooter Rotation D";
    m_rotationGKey = "Shooter Rotation G";
    m_rotationSKey = "Shooter Rotation S";
    m_rotationVKey = "Shooter Rotation V";
    m_rotationAKey = "Shooter Rotation A";
    m_rotationTargetKey = "Shooter Rotation Target";

    frc::Preferences::SetDouble(m_rotationPKey, ShooterConstants::kPRotation);
    frc::Preferences::SetDouble(m_rotationIKey, ShooterConstants::kIRotation);
    frc::Preferences::SetDouble(m_rotationDKey, ShooterConstants::kDRotation);
    frc::Preferences::SetDouble(m_rotationGKey, ShooterConstants::kGRotation.value());
    frc::Preferences::SetDouble(m_rotationSKey, ShooterConstants::kSRotation.value());
    frc::Preferences::SetDouble(m_rotationVKey, ShooterConstants::kVRotation.value());
    frc::Preferences::SetDouble(m_rotationAKey, ShooterConstants::kARotation.value());
    frc::Preferences::SetDouble(m_rotationTargetKey, m_target.value());

    m_rotationPIDController.SetFeedbackDevice(m_rotationEncoder);

    m_rotationPIDController.SetP(kRotationTargetPID[ShooterState::Retracted][0]);
    m_rotationPIDController.SetI(kRotationTargetPID[ShooterState::Retracted][1]);
    m_rotationPIDController.SetD(kRotationTargetPID[ShooterState::Retracted][2]);

    m_rotationPIDController.SetP(kRotationTargetPID[ShooterState::Load][0], 1);
    m_rotationPIDController.SetI(kRotationTargetPID[ShooterState::Load][1], 1);
    m_rotationPIDController.SetD(kRotationTargetPID[ShooterState::Load][2], 1);
}

void Shooter::UpdatePreferences() {
    m_profiledPIDController.SetP(frc::Preferences::GetDouble(m_rotationPKey, ShooterConstants::kPRotation));
    m_profiledPIDController.SetI(frc::Preferences::GetDouble(m_rotationIKey, ShooterConstants::kIRotation));
    m_profiledPIDController.SetD(frc::Preferences::GetDouble(m_rotationDKey, ShooterConstants::kDRotation));
    double s = frc::Preferences::GetDouble(m_rotationSKey, ShooterConstants::kSRotation.value());
    double g = frc::Preferences::GetDouble(m_rotationGKey, ShooterConstants::kGRotation.value());
    double v = frc::Preferences::GetDouble(m_rotationVKey, ShooterConstants::kVRotation.value());
    double a = frc::Preferences::GetDouble(m_rotationAKey, ShooterConstants::kARotation.value());
    m_target = units::degree_t{frc::Preferences::GetDouble(m_rotationTargetKey, m_target.value())};
    delete m_ff;
    m_ff = new frc::ArmFeedforward(
        units::volt_t{s},
        units::volt_t{g},
        units::unit_t<frc::ArmFeedforward::kv_unit>{v},
        units::unit_t<frc::ArmFeedforward::ka_unit>{a}
    );
}

frc2::CommandPtr Shooter::SysIdQuasistatic(frc2::sysid::Direction direction) {
    // for (int i = 0; i < 10; i++) 
    //     std::cout << "Shooter Quasistatic\n";
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Shooter::SysIdDynamic(frc2::sysid::Direction direction) {
    // for (int i = 0; i < 10; i++) 
    //     std::cout << "Shooter Quasistatic\n";
  return m_sysIdRoutine.Dynamic(direction);
}

void Shooter::SetState(ShooterState state, units::degree_t autoAlignAngle){
    auto target = 0.0_deg;

    switch(state){
        case(ShooterState::None):
            break;
        case(ShooterState::Retracted):
            target = ShooterConstants::kRetractTarget;
            m_currentState = ShooterState::Retracted;
            break;
        case(ShooterState::Load):
            target = ShooterConstants::kLoadTarget;
            m_currentState = ShooterState::Load;
            break;
        case(ShooterState::DirectLoad):
            target = ShooterConstants::kDirectLoadTarget;
            m_currentState = ShooterState::DirectLoad;
            break;
        case(ShooterState::Close):
            target = ShooterConstants::kCloseTarget;
            m_currentState = ShooterState::Close;
            break;
        case(ShooterState::Mid):
            target = ShooterConstants::kMidTarget;
            m_currentState = ShooterState::Mid;
            break;
        case(ShooterState::Far):
            target = ShooterConstants::kFarTarget;
            m_currentState = ShooterState::Far;
            break;
        case(ShooterState::Zero):
            target = 0_deg;
            m_currentState = ShooterState::Zero;
            break;
        case(ShooterState::AutoAlign):
            target = autoAlignAngle;
            m_currentState = ShooterState::AutoAlign;
            break;
        case(ShooterState::StartingConfig):
            target = ShooterConstants::kStartingConfigTarget;
            m_currentState = ShooterState::StartingConfig;
            break;
        case(ShooterState::AutoScore):
            target = ShooterConstants::kAutoScoreTarget;
            m_currentState = ShooterState::AutoScore;
        default: 
            break;
    }

    m_target = target;
}

units::degree_t Shooter::GetTarget() {
    return m_target; 
}

void Shooter::SetActive(bool active) {
    if (active) {
        m_lastTime = frc::Timer::GetFPGATimestamp();
    }

    m_isActive = active;
}

bool Shooter::NoteDetected(){
    if (m_limitSwitchRight.Get() || m_limitSwitchLeft.Get()){
        return true;
    }
    return false;
}

void Shooter::SetBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_rollerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_loaderMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            break;
        case(BrakeMode::Coast) :
            m_rollerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_loaderMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            break;
        case(BrakeMode::Default) :
            m_rollerMotor.SetIdleMode(ShooterConstants::kIdleMode);
            m_rotationMotor.SetIdleMode(ShooterConstants::kIdleMode);
            m_loaderMotor.SetIdleMode(ShooterConstants::kIdleMode);
            break;
        default :
            break;
    }
}