// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "util/SwerveModule.h"

SwerveModule::SwerveModule(int driveMotorID, int angleMotorID,
                           int absoluteEncoderID, double absoluteEncoderOffset):
m_driveMotor(driveMotorID, "Swerve"),
m_angleMotor(angleMotorID, "Swerve"),
m_absoluteEncoder(absoluteEncoderID, "Swerve"),
m_absoluteEncoderOffset(absoluteEncoderOffset) {
    ConfigDriveMotor();
    ConfigAngleMotor(absoluteEncoderID);
    ConfigEncoder();
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

void SwerveModule::ConfigAngleMotor(int absoluteEncoderID) {
    // Set to factory default
    m_angleMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));
    ctre::phoenix6::configs::TalonFXConfiguration angleConfigs{};

    angleConfigs.Slot0.kP = SwerveModuleConstants::kPAngle;
    angleConfigs.Slot0.kI = SwerveModuleConstants::kIAngle;
    angleConfigs.Slot0.kD = SwerveModuleConstants::kDAngle;
    angleConfigs.Slot0.kV = SwerveModuleConstants::kVAngle;
    angleConfigs.Slot0.kS = SwerveModuleConstants::kSAngle;

    angleConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    // angleConfigs.Feedback.SensorToMechanismRatio = SwerveModuleConstants::kAngleGearRatio;
    // TODO: Not sure if this number is correct/if it actually works this way on our modules
    angleConfigs.Feedback.RotorToSensorRatio = SwerveModuleConstants::kAngleGearRatio;
    angleConfigs.Feedback.FeedbackRemoteSensorID = absoluteEncoderID;
    angleConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;

    angleConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants::kAngleContinuousCurrentLimit;
    angleConfigs.CurrentLimits.SupplyCurrentThreshold = SwerveModuleConstants::kAnglePeakCurrentLimit;
    angleConfigs.CurrentLimits.SupplyTimeThreshold = SwerveModuleConstants::kAnglePeakCurrentDuration;
    angleConfigs.CurrentLimits.SupplyCurrentLimitEnable = SwerveModuleConstants::kAngleEnableCurrentLimit;
    
    m_angleMotor.SetInverted(SwerveModuleConstants::kAngleMotorInverted);
    m_angleMotor.SetNeutralMode(SwerveModuleConstants::kAngleNeutralMode);

    m_angleMotor.GetConfigurator().Apply(angleConfigs);
}

void SwerveModule::ConfigEncoder() {
    // Set to factory default
    m_absoluteEncoder.GetConfigurator().Apply(ctre::phoenix6::configs::CANcoderConfiguration{});
    ctre::phoenix6::configs::CANcoderConfiguration encoderConfigs{};

    // Set the magnet offset in configs
    encoderConfigs.MagnetSensor.MagnetOffset = m_absoluteEncoderOffset;
    encoderConfigs.MagnetSensor.SensorDirection = SwerveModuleConstants::;
    encoderConfigs.MagnetSensor.AbsoluteSensorRange = SwerveModuleConstants::kAbsoluteEncoderSensorRange;

    m_absoluteEncoder.GetConfigurator().Apply(encoderConfigs);
}

void SwerveModule::Stop() {
    m_driveMotor.StopMotor();
    m_angleMotor.StopMotor();
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &state) {
    const auto state = frc::SwerveModuleState::Optimize(state, frc::Rotation2d{units::degree_t{m_absoluteEncoder.GetPosition()}});
}