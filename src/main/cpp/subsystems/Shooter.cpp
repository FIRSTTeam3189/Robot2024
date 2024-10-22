// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter(PoseEstimatorHelper *estimator) : 
m_topRollerMotor(ShooterConstants::kTopRollerMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_bottomRollerMotor(ShooterConstants::kBottomRollerMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_rollerEncoder(m_topRollerMotor.GetEncoder()),
m_loaderMotor(ShooterConstants::kLoaderMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_rotationMotor(ShooterConstants::kRotationMotorID, rev::CANSparkMax::MotorType::kBrushless),
m_limitSwitchLeft(ShooterConstants::kLeftLimitSwitchPort),
m_limitSwitchRight(ShooterConstants::kRightLimitSwitchPort),
m_alignUtil(estimator),
m_constraints(ShooterConstants::kMaxRotationVelocity, ShooterConstants::kMaxRotationAcceleration),
m_profiledPIDController(ShooterConstants::kPRotation, ShooterConstants::kIRotation, ShooterConstants::kDRotation, m_constraints),
m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)), 
m_target(0.0_deg),
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
    frc::SmartDashboard::PutNumber("Shooter rotation", GetRotation().value());
    frc::SmartDashboard::PutNumber("Shooter power", m_topRollerMotor.Get());
    frc::SmartDashboard::PutNumber("Shooter RPM", m_rollerEncoder.GetVelocity());
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

    // TODO: remove after testing
    m_alignUtil.GetShooterGoalInterpolating(m_alignUtil.GetDistanceToSpeaker());

    //constructed all instances of variables needed as well as keys for PID and diagnostic values
    // by default holds position but if it is active then it goes to target
}

void Shooter::HoldPosition(units::degree_t target) {
    units::volt_t PIDValue = 0.0_V;
    if (fabs(m_target.value() - GetRotation().value()) > ShooterConstants::kRotationIdleTolerance.value()) {
        PIDValue = units::volt_t{(m_target - GetRotation()).value() * m_profiledPIDController.GetP()};
    } else {
        m_rotationMotor.StopMotor();
        return;
    }

    auto ffValue = m_ff->Calculate(units::radian_t{GetRotation()}, units::radians_per_second_t{0.0});
    frc::SmartDashboard::PutNumber("Shooter rotation FF", ffValue.value());
    frc::SmartDashboard::PutNumber("Shooter rotation PID", PIDValue.value());
    m_rotationMotor.SetVoltage(std::clamp((ffValue + PIDValue), -12.0_V, 12.0_V));
    // Just to advance the profile timestep
    m_profiledPIDController.Calculate(GetRotation(), m_target);

    //same logic as intake
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

    //  if (GetRotation() < 10.0_deg && (PIDValue + ffValue).value() <= 0.0)
    //     m_rotationMotor.SetVoltage(0.0_V);
    // else if (GetRotation() > 60.0_deg && (PIDValue + ffValue).value() >= 0.0)
    //     m_rotationMotor.SetVoltage(0.0_V);
    // else {
    if (GetRotation() > 60.0_deg && (PIDValue + ffValue).value() >= 0.0) {
        m_rotationMotor.SetVoltage(0.0_V);
    } else {
        m_rotationMotor.SetVoltage(std::clamp((PIDValue + ffValue), -12.0_V, 12.0_V));
    }
    // }
}

void Shooter::SetRollerPower(double power) {
    m_topRollerMotor.Set(power);
    m_bottomRollerMotor.Set(power);
}

void Shooter::SetRotationPower(double power) {
    m_rotationMotor.Set(power);
}

void Shooter::SetLoaderPower(double power) {
    m_loaderMotor.Set(power);
}

units::degree_t Shooter::GetRotation() {
    auto angle = units::degree_t{m_rotationEncoder.GetPosition()};
    if(angle >= 350_deg) {
        angle = 0.0_deg;
    }
    return angle;
}

void Shooter::ConfigRollerMotor() {
    m_topRollerMotor.RestoreFactoryDefaults();
    m_topRollerMotor.SetSmartCurrentLimit(ShooterConstants::kRollerCurrentLimit);
    m_topRollerMotor.SetInverted(ShooterConstants::kRollerInverted);
    m_bottomRollerMotor.RestoreFactoryDefaults();
    m_bottomRollerMotor.SetSmartCurrentLimit(ShooterConstants::kRollerCurrentLimit);
    m_bottomRollerMotor.SetInverted(!ShooterConstants::kRollerInverted);
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

    //constructs PID and FF objects using the keys and values needed to be set.
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

    //updates PID and FF values with newest ones constructed
}

void Shooter::SetState(ShooterState state, units::degree_t autoAlignAngle){
    auto target = 0.0_deg;

    switch(state){
        case(ShooterState::None):
            break;
        case(ShooterState::Retracted):
            target = ShooterConstants::kRetractTarget;
            break;
        case(ShooterState::Load):
            target = ShooterConstants::kLoadTarget;
            break;
        case(ShooterState::DirectLoad):
            target = ShooterConstants::kDirectLoadTarget;
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
        case(ShooterState::Zero):
            target = 0.0_deg;
            break;
        case(ShooterState::AutoAlign):
            target = autoAlignAngle;
            break;
        case(ShooterState::StartingConfig):
            target = ShooterConstants::kStartingConfigTarget;
            break;
        case(ShooterState::AutoScore):
            target = ShooterConstants::kAutoScoreTarget;
            SetRollerPower(ShooterConstants::kShootPower);
            break;
        case(ShooterState::ArbitraryAngle):
            // Will set target to the arbitrary angle based on UpdatePreferences()
            target = units::degree_t{frc::Preferences::GetDouble(m_rotationTargetKey, m_target.value())};
            break;
        case(ShooterState::InterpolateAngle):
            // TODO: make it actually shoot
            // This will set to the angle calculated by the interpolation algorithm
            // target = m_alignUtil.GetShooterGoalInterpolating(m_alignUtil.GetDistanceToSpeaker());
            m_alignUtil.GetShooterGoalInterpolating(m_alignUtil.GetDistanceToSpeaker());
            break;
        default: 
            break;
    }

    //states for different scenarios on the shooter and sets target to the needed one for the possibilities

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
    if (!m_limitSwitchRight.Get() || !m_limitSwitchLeft.Get()){
        frc::SmartDashboard::PutBoolean("Shooter note detected", true);
        return true;
    } else {
        frc::SmartDashboard::PutBoolean("Shooter note detected", false);
        return false;
    } 
}

void Shooter::SetBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_topRollerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_bottomRollerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_loaderMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            break;
        case(BrakeMode::Coast) :
            m_topRollerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_bottomRollerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_loaderMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            break;
        case(BrakeMode::Default) :
            m_topRollerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_bottomRollerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_rotationMotor.SetIdleMode(ShooterConstants::kIdleMode);
            m_loaderMotor.SetIdleMode(ShooterConstants::kIdleMode);
            break;
        default :
            break;
    }
}