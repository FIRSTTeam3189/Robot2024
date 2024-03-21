// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() : 
 m_rotationMotor(IntakeConstants::kRotationMotorID, rev::CANSparkMax::MotorType::kBrushless),
 m_rollerMotor(IntakeConstants::kRollerMotorID, rev::CANSparkMax::MotorType::kBrushless),
 m_limitSwitchLeft(IntakeConstants::kLeftLimitSwitchPort),
 m_limitSwitchRight(IntakeConstants::kRightLimitSwitchPort),
 m_constraints(IntakeConstants::kMaxRotationVelocity, IntakeConstants::kMaxRotationAcceleration),
 m_profiledPIDController(IntakeConstants::kPRotation, IntakeConstants::kIRotation, IntakeConstants::kDRotation, m_constraints),
 m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
 m_target(IntakeConstants::kRetractTarget),
 m_bill(OperatorConstants::kDriverControllerPort),
m_isActive(false)
{
    ConfigRotationMotor();
    ConfigRollerMotor();
    ConfigPID();

    //configure motors and PID

    // std::cout << "Intake constructed\n";
}

void Intake::ConfigRotationMotor() {
    m_rotationMotor.RestoreFactoryDefaults();
    m_rotationMotor.SetIdleMode(IntakeConstants::kIdleMode);
    m_rotationMotor.SetSmartCurrentLimit(IntakeConstants::kRotationCurrentLimit);
    m_rotationMotor.SetInverted(IntakeConstants::kRotationMotorInverted);
    m_rotationMotor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus5, 20);
    m_rotationMotor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus6, 20);
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

    //takes speed, gravity, velocity and acceleration to construct feed forward object

    m_rotationPKey = "Intake Rotation P";
    m_rotationIKey = "Intake Rotation I";
    m_rotationDKey = "IntakeSetState Rotation D";
    m_rotationGKey = "Intake Rotation G";
    m_rotationSKey = "Intake Rotation S";
    m_rotationVKey = "Intake Rotation V";
    m_rotationAKey = "Intake Rotation A";
    m_rotationTargetKey = "Intake Rotation Target";
    //keys for PID and FF 

    frc::Preferences::SetDouble(m_rotationPKey, IntakeConstants::kPRotation);
    frc::Preferences::SetDouble(m_rotationIKey, IntakeConstants::kIRotation);
    frc::Preferences::SetDouble(m_rotationDKey, IntakeConstants::kDRotation);
    frc::Preferences::SetDouble(m_rotationGKey, IntakeConstants::kGRotation.value());
    frc::Preferences::SetDouble(m_rotationSKey, IntakeConstants::kSRotation.value());
    frc::Preferences::SetDouble(m_rotationVKey, IntakeConstants::kVRotation.value());
    frc::Preferences::SetDouble(m_rotationAKey, IntakeConstants::kARotation.value());
    frc::Preferences::SetDouble(m_rotationTargetKey, m_target.value());
}

//initializing PID and SGVA values

// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake PID target", m_target.value());
    frc::SmartDashboard::PutNumber("Intake rotation", GetRotation().value());
    frc::SmartDashboard::PutNumber("Intake power", m_rollerMotor.Get());

    if (frc::Preferences::GetBoolean("Full Diagnostics", false)) {
        frc::SmartDashboard::PutNumber("Intake rotational velocity", m_rotationEncoder.GetVelocity());
        frc::SmartDashboard::PutNumber("Intake desired rotational velocity", m_profiledPIDController.GetSetpoint().velocity.value());
        frc::SmartDashboard::PutNumber("Intake rotational acceleration", m_acceleration.value());
        frc::SmartDashboard::PutNumber("Intake desired rotational acceleration", m_targetAcceleration.value());
    }

    if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
        UpdatePreferences();
    }

    if (m_isActive) {
        SetRotation(m_target);
    } else {
        HoldPosition();
    }
}

void Intake::SetActive(bool active) {
    if (active) {
        m_lastTime = frc::Timer::GetFPGATimestamp();
    }

    m_isActive = active;
}

bool Intake::NoteDetected(){
    if (!m_limitSwitchRight.Get() || !m_limitSwitchLeft.Get()){
        frc::SmartDashboard::PutBoolean("Intake note detected", true);
        m_bill.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
        return true;
    } else {
        frc::SmartDashboard::PutBoolean("Intake note detected", false);
        return false;
    } 
}

// Limit switches to detect of the note is there

units::degree_t Intake::GetTarget() {
    return m_target;
}
//Allows the desired target to be accessed by other commands

void Intake::SetState(IntakeState state) {
    auto target = 0.0_deg;

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

    //states of the amp

    m_target = target;
    //update the m_target variable with the target value changed from the state
}

void Intake::HoldPosition() {
    units::volt_t PIDValue = 0.0_V;
    if (fabs(m_target.value() - GetRotation().value()) > IntakeConstants::kRotationIdleTolerance.value()) {
        PIDValue = units::volt_t{(m_target - GetRotation()).value() * m_profiledPIDController.GetP()};
        //uses proportional in PID to account for changes of positioning based on desired position to hold
    } else {
        m_rotationMotor.StopMotor();
        return;
    }

    auto ffValue = m_ff->Calculate(units::radian_t{GetRotation()}, units::radians_per_second_t{0.0});
    frc::SmartDashboard::PutNumber("Intake rotation FF", ffValue.value());
    frc::SmartDashboard::PutNumber("Intake rotation PID", PIDValue.value());
    m_rotationMotor.SetVoltage(std::clamp((ffValue + PIDValue), -12.0_V, 12.0_V));
    // Just to advance the profile timestep
    m_profiledPIDController.Calculate(GetRotation(), m_target);
}

//uses PID and FF value to hold the given position. Accounts for gravity and desired position as well as start position to the same rotation

void Intake::SetRotation(units::degree_t target) {
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
    // if (GetRotation() <= -35.0_deg && (PIDValue + ffValue).value() <= 0.0)
    //     m_rotationMotor.SetVoltage(0.0_V);
    // else if (GetRotation() >= 85.0_deg && (PIDValue + ffValue).value() >= 0.0)
    //     m_rotationMotor.SetVoltage(0.0_V);
    // else {
        m_rotationMotor.SetVoltage(std::clamp((PIDValue + ffValue), -12.0_V, 12.0_V));
    // }

    frc::SmartDashboard::PutNumber("Intake power", m_rotationMotor.Get());
    frc::SmartDashboard::PutNumber("Intake rotation volts", PIDValue.value() + ffValue.value());
    frc::SmartDashboard::PutNumber("Intake rotation PID", PIDValue.value());
    frc::SmartDashboard::PutNumber("Intake rotation FF", ffValue.value());

    m_lastTargetSpeed = m_profiledPIDController.GetSetpoint().velocity;
    m_lastSpeed = units::degrees_per_second_t{m_rotationEncoder.GetVelocity()};
    m_lastTime = frc::Timer::GetFPGATimestamp();
}

void Intake::SetRollerPower(double power) {
    m_rollerMotor.Set(power);
}

void Intake::SetRotationPower(double power) {
    if (GetRotation() <= -35.0_deg && power <= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else if (GetRotation() >= 85.0_deg && power >= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else
        m_rotationMotor.Set(power);
}

//Sets voltage of the rotation motors to 0.0 if in the case the motors go to far so it protects from further damages

units::degree_t Intake::GetRotation() {
    auto rawRotation = m_rotationEncoder.GetPosition();
    if (rawRotation > 180.0)
        rawRotation -= 360.0;

    return units::degree_t{rawRotation};
}

void Intake::UpdatePreferences() {
    m_profiledPIDController.SetP(frc::Preferences::GetDouble(m_rotationPKey, IntakeConstants::kPRotation));
    m_profiledPIDController.SetI(frc::Preferences::GetDouble(m_rotationIKey, IntakeConstants::kIRotation));
    m_profiledPIDController.SetD(frc::Preferences::GetDouble(m_rotationDKey, IntakeConstants::kDRotation));
    // set the PID values based on the inputted key
    double s = frc::Preferences::GetDouble(m_rotationSKey, IntakeConstants::kSRotation.value());
    double g = frc::Preferences::GetDouble(m_rotationGKey, IntakeConstants::kGRotation.value());
    double v = frc::Preferences::GetDouble(m_rotationVKey, IntakeConstants::kVRotation.value());
    double a = frc::Preferences::GetDouble(m_rotationAKey, IntakeConstants::kARotation.value());
    //get the speed, gravity, velocity and acceleration values
    m_target = units::degree_t{frc::Preferences::GetDouble(m_rotationTargetKey, m_target.value())};
    delete m_ff;
    m_ff = new frc::ArmFeedforward(
        units::volt_t{s},
        units::volt_t{g},
        units::unit_t<frc::ArmFeedforward::kv_unit>{v},
        units::unit_t<frc::ArmFeedforward::ka_unit>{a}
        //constructs the updated feed forward object
    );
}

void Intake::SetBrakeMode(BrakeMode mode) {
    switch (mode) {
        case(BrakeMode::Brake) :
            m_rollerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            break;
        case(BrakeMode::Coast) :
            m_rollerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            m_rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            break;
        case(BrakeMode::Default) :
            m_rollerMotor.SetIdleMode(IntakeConstants::kIdleMode);
            m_rotationMotor.SetIdleMode(IntakeConstants::kIdleMode);
            break;
        default :
            break;
    }
}