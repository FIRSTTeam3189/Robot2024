// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() : 
 m_rotationMotor(IntakeConstants::kRotationMotorID, rev::CANSparkMax::MotorType::kBrushless),
 m_rollerMotor(IntakeConstants::kRollerMotorID, rev::CANSparkMax::MotorType::kBrushless),
 m_limitSwitch(IntakeConstants::kLimitSwitchPort),
 m_constraints(IntakeConstants::kMaxRotationVelocity, IntakeConstants::kMaxRotationAcceleration),
 m_profiledPIDController(IntakeConstants::kPRotation, IntakeConstants::kIRotation, IntakeConstants::kDRotation, m_constraints),
 m_rotationPIDController(m_rotationMotor.GetPIDController()),
 m_rotationEncoder(m_rotationMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
 m_target(IntakeConstants::kRetractTarget - 0.1_deg),
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
m_prevState(IntakeState::None),
m_isActive(false),
m_noteState(NoteState::None),
m_lastNoteState(NoteState::None)
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

    m_rotationPKey = "Intake Rotation P";
    m_rotationIKey = "Intake Rotation I";
    m_rotationDKey = "Intake Rotation D";
    m_rotationGKey = "Intake Rotation G";
    m_rotationSKey = "Intake Rotation S";
    m_rotationVKey = "Intake Rotation V";
    m_rotationAKey = "Intake Rotation A";
    m_rotationTargetKey = "Intake Rotation Target";

    frc::Preferences::SetDouble(m_rotationPKey, IntakeConstants::kPRotation);
    frc::Preferences::SetDouble(m_rotationIKey, IntakeConstants::kIRotation);
    frc::Preferences::SetDouble(m_rotationDKey, IntakeConstants::kDRotation);
    frc::Preferences::SetDouble(m_rotationGKey, IntakeConstants::kGRotation.value());
    frc::Preferences::SetDouble(m_rotationSKey, IntakeConstants::kSRotation.value());
    frc::Preferences::SetDouble(m_rotationVKey, IntakeConstants::kVRotation.value());
    frc::Preferences::SetDouble(m_rotationAKey, IntakeConstants::kARotation.value());
    frc::Preferences::SetDouble(m_rotationTargetKey, m_target.value());

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
    frc::SmartDashboard::PutNumber("Intake rotation", GetRotation().value());
    UpdateUltrasonic();
    UpdateNoteState();

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

units::degree_t Intake::GetTarget() {
    return m_target;
}

void Intake::SetState(IntakeState state) {
    auto target = 0.0_deg;

    switch(state){
        case (IntakeState::None) :
            break;
        case (IntakeState::Extended) :
            target = IntakeConstants::kExtendTarget;
            m_currentState = IntakeState::Extended;
            break;
        case (IntakeState::Amp) :
            target = IntakeConstants::kAmpTarget;
            m_currentState = IntakeState::Amp;
            break;
        case (IntakeState::Retracted) :
            target = IntakeConstants::kRetractTarget;
            m_currentState = IntakeState::Retracted;
            break;
    }

    m_target = target;
}

void Intake::HoldPosition() {
    auto ffValue = m_ff->Calculate(units::radian_t{GetRotation()}, units::radians_per_second_t{0.0});
    m_rotationMotor.SetVoltage(std::clamp((ffValue), -12.0_V, 12.0_V));
    // Just to advance the profile timestep
    m_profiledPIDController.Calculate(GetRotation(), m_target);
}

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
    if (GetRotation() <= -35.0_deg && (PIDValue + ffValue).value() <= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else if (GetRotation() >= 85.0_deg && (PIDValue + ffValue).value() >= 0.0)
        m_rotationMotor.SetVoltage(0.0_V);
    else {
        m_rotationMotor.SetVoltage(std::clamp((PIDValue + ffValue), -12.0_V, 12.0_V));
    }

    frc::SmartDashboard::PutNumber("Intake rotation volts", PIDValue.value() + ffValue.value());
    frc::SmartDashboard::PutNumber("Intake rotation PID", PIDValue.value());
    frc::SmartDashboard::PutNumber("Intake rotation FF", ffValue.value());

    m_lastTargetSpeed = m_profiledPIDController.GetSetpoint().velocity;
    m_lastSpeed = units::degrees_per_second_t{m_rotationEncoder.GetVelocity()};
    m_lastTime = frc::Timer::GetFPGATimestamp();

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

        // if (m_currentState == IntakeState::Extended && m_prevstate == IntakeState::Retracted) {
        //     // m_rotationPIDController.SetReference(target.value(), rev::ControlType::kPosition, 0, ff);
        // } else if (m_currentState == IntakeState::Retracted && m_prevstate == IntakeState::Extended) {
        //     // m_rotationPIDController.SetReference(target.value(), rev::ControlType::kPosition, 1, ff);
        // } else {
        //     m_rotationPIDController.SetReference(target.value(), rev::ControlType::kPosition, 2, ff);
        // }
    // }
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
    auto rawRotation = m_rotationEncoder.GetPosition();
    if (rawRotation > 180.0)
        rawRotation -= 360.0;

    return units::degree_t{rawRotation};
}

void Intake::UpdatePreferences() {
    m_profiledPIDController.SetP(frc::Preferences::GetDouble(m_rotationPKey, IntakeConstants::kPRotation));
    m_profiledPIDController.SetI(frc::Preferences::GetDouble(m_rotationIKey, IntakeConstants::kIRotation));
    m_profiledPIDController.SetD(frc::Preferences::GetDouble(m_rotationDKey, IntakeConstants::kDRotation));
    double s = frc::Preferences::GetDouble(m_rotationSKey, IntakeConstants::kSRotation.value());
    double g = frc::Preferences::GetDouble(m_rotationGKey, IntakeConstants::kGRotation.value());
    double v = frc::Preferences::GetDouble(m_rotationVKey, IntakeConstants::kVRotation.value());
    double a = frc::Preferences::GetDouble(m_rotationAKey, IntakeConstants::kARotation.value());
    m_target = units::degree_t{frc::Preferences::GetDouble(m_rotationTargetKey, m_target.value())};
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

void Intake::UpdateNoteState() {
    bool detected = !m_limitSwitch.Get();
    m_lastNoteState = m_noteState;
    switch (m_noteState) {
        case (NoteState::None) :
            if (detected) {
                m_noteState = NoteState::FirstDetection;
            }
            frc::SmartDashboard::PutString("Intake note state", "None");
            break;
        case (NoteState::FirstDetection) :
            if (!detected) {
                m_noteState = NoteState::MiddleOfNote;
            }
            frc::SmartDashboard::PutString("Intake note state", "First detection");
            break;
        case (NoteState::MiddleOfNote) :
            if (detected) {
                m_noteState = NoteState::SecondDetection;
            }
            frc::SmartDashboard::PutString("Intake note state", "Middle of note");
            break;
        case (NoteState::SecondDetection) :
            if (!detected) {
                m_noteState = NoteState::None;
            }
            frc::SmartDashboard::PutString("Intake note state", "Second detection");
            break;
        default :
            frc::SmartDashboard::PutString("Intake note state", "None");
            break;
    }
}

NoteState Intake::GetNoteState() {
    return m_noteState;
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