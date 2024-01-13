// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/SwerveModule.h"

SwerveModule::SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
                           int CANcoderID, double CANcoderOffset):
m_driveMotor(driveMotorID, "Swerve"),
m_angleMotor(angleMotorID, "Swerve"),
m_CANcoder(CANcoderID, "Swerve"),
m_moduleNumber(moduleNumber),
m_CANcoderOffset(CANcoderOffset),
m_signals{m_drivePosition, m_anglePosition, m_driveVelocity, m_angleVelocity}
 {
    ConfigDriveMotor();
    ConfigAngleMotor(CANcoderID);
    ConfigCANcoder();
}

void SwerveModule::ConfigDriveMotor() {
    // Set to factory default
    m_driveMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});
    ctre::phoenix6::configs::TalonFXConfiguration driveConfigs{};

    driveConfigs.Slot0.kP = SwerveModuleConstants::kPDrive;
    driveConfigs.Slot0.kI = SwerveModuleConstants::kIDrive;
    driveConfigs.Slot0.kD = SwerveModuleConstants::kDDrive;
    driveConfigs.Slot0.kV = SwerveModuleConstants::kVDrive;
    driveConfigs.Slot0.kS = SwerveModuleConstants::kSDrive;

    driveConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants::kDriveContinuousCurrentLimit;
    driveConfigs.CurrentLimits.SupplyCurrentThreshold = SwerveModuleConstants::kDrivePeakCurrentLimit;
    driveConfigs.CurrentLimits.SupplyTimeThreshold = SwerveModuleConstants::kDrivePeakCurrentDuration;
    driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = SwerveModuleConstants::kDriveEnableCurrentLimit;

    driveConfigs.MotorOutput.Inverted = SwerveModuleConstants::kDriveMotorInverted;
    driveConfigs.MotorOutput.NeutralMode = SwerveModuleConstants::kDriveNeutralMode;
    
    driveConfigs.Feedback.SensorToMechanismRatio = SwerveModuleConstants::kDriveGearRatio;

    m_driveMotor.GetConfigurator().Apply(driveConfigs);

    m_driveMotor.SetPosition(0.0_rad);
}

void SwerveModule::ConfigAngleMotor(int CANcoderID) {
    // Set to factory default
    m_angleMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));
    ctre::phoenix6::configs::TalonFXConfiguration angleConfigs{};

    angleConfigs.Slot0.kP = SwerveModuleConstants::kPAngle;
    angleConfigs.Slot0.kI = SwerveModuleConstants::kIAngle;
    angleConfigs.Slot0.kD = SwerveModuleConstants::kDAngle;
    angleConfigs.Slot0.kV = SwerveModuleConstants::kVAngle;
    angleConfigs.Slot0.kS = SwerveModuleConstants::kSAngle;

    angleConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    angleConfigs.Feedback.SensorToMechanismRatio = SwerveModuleConstants::kAngleGearRatio;
    // TODO: Not sure if this number is correct/if it actually works this way on our modules
    // angleConfigs.Feedback.RotorToSensorRatio = SwerveModuleConstants::kAngleGearRatio;
    // angleConfigs.Feedback.FeedbackRemoteSensorID = CANcoderID;
    // angleConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;

    angleConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants::kAngleContinuousCurrentLimit;
    angleConfigs.CurrentLimits.SupplyCurrentThreshold = SwerveModuleConstants::kAnglePeakCurrentLimit;
    angleConfigs.CurrentLimits.SupplyTimeThreshold = SwerveModuleConstants::kAnglePeakCurrentDuration;
    angleConfigs.CurrentLimits.SupplyCurrentLimitEnable = SwerveModuleConstants::kAngleEnableCurrentLimit;
    
    m_angleMotor.SetInverted(SwerveModuleConstants::kAngleMotorInverted);
    m_angleMotor.SetNeutralMode(SwerveModuleConstants::kAngleNeutralMode);

    m_angleMotor.GetConfigurator().Apply(angleConfigs);
}

void SwerveModule::ConfigCANcoder() {
    // Set to factory default
    m_CANcoder.GetConfigurator().Apply(ctre::phoenix6::configs::CANcoderConfiguration{});
    ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{};

    // Set the magnet offset in configs
    encoderConfigs.MagnetSensor.MagnetOffset = m_CANcoderOffset;
    encoderConfigs.MagnetSensor.SensorDirection = SwerveModuleConstants::kCANcoderInverted;
    encoderConfigs.MagnetSensor.AbsoluteSensorRange = SwerveModuleConstants::kCANcoderSensorRange;

    m_CANcoder.GetConfigurator().Apply(encoderConfigs);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &state) {
    const auto optimizedState = frc::SwerveModuleState::Optimize(state, m_position.angle);
    double targetSpeed = optimizedState.speed.value();
    auto targetAngle = optimizedState.angle.Degrees();

    frc::SmartDashboard::PutNumber(std::string("" + m_moduleNumber) + " target speed", targetSpeed);
    frc::SmartDashboard::PutNumber(std::string("" + m_moduleNumber) + " target angle", targetAngle.value());

    m_driveMotor.SetControl(m_driveSetter.WithVelocity(units::turns_per_second_t{targetSpeed}));
    m_angleMotor.SetControl(m_angleSetter.WithPosition(targetAngle));
}

Signals SwerveModule::GetSignals() {
    return m_signals;
}

void SwerveModule::Stop() {
    m_driveMotor.StopMotor();
    m_angleMotor.StopMotor();
}

void SwerveModule::UpdatePosition() {
    m_drivePosition.Refresh();
    m_driveVelocity.Refresh();
    m_anglePosition.Refresh();
    m_angleVelocity.Refresh();

    auto driveRotations = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
    auto angleRotations = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(m_anglePosition, m_angleVelocity);

    // Have to convert rotations to double then to meters with our own rotation coefficient
    double distance = driveRotations.value() / SwerveModuleConstants::kRotationsPerMeter;
    m_position.distance = units::meter_t{distance};
    frc::Rotation2d angle{units::degree_t{angleRotations}};
    m_position.angle = angle;

    frc::SmartDashboard::PutNumber(std::string("" + m_moduleNumber) + " distance", distance);
    frc::SmartDashboard::PutNumber(std::string("" + m_moduleNumber) + " angle", angle.Degrees().value());
}

frc::SwerveModulePosition SwerveModule::GetPosition(bool refresh) {
    if (refresh)
        UpdatePosition();

    return m_position;
}

frc::SwerveModuleState SwerveModule::GetState(bool refresh) {
    if (refresh)
        UpdatePosition();

    // Uses initializer list syntax and lets compiler make swerve module state since we can't construct directly
    // Note the 360 converting rotations to degrees, and the turns per second to meters per second
    return {units::meters_per_second_t{m_driveVelocity.GetValue().value() / SwerveModuleConstants::kRotationsPerMeter}, 
            frc::Rotation2d(units::degree_t{360 * m_anglePosition.GetValue().value()})};
}

units::degree_t SwerveModule::GetMotorAngle() {
    return units::degree_t{m_angleMotor.GetPosition().Refresh().GetValue()};
}

units::degree_t SwerveModule::GetEncoderAngle() {
    return units::degree_t{m_CANcoder.GetAbsolutePosition().Refresh().GetValue()};
}

units::meters_per_second_t SwerveModule::GetDriveSpeed() {
    return units::meters_per_second_t{m_driveMotor.GetVelocity().Refresh().GetValue().value() / SwerveModuleConstants::kRotationsPerMeter};
}