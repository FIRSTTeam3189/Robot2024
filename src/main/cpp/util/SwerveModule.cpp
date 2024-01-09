// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "util/SwerveModule.h"

SwerveModule::SwerveModule(int speedMotorID, int angleMotorID, bool speedMotorInverted, bool angleMotorInverted,
                           int absoluteEncoderID, double absoluteEncoderOffset, bool absoluteEncoderInverted):
m_speedMotor(speedMotorID, "Swerve"),
m_angleMotor(angleMotorID, "Swerve"),
m_absoluteEncoder(absoluteEncoderID, "Swerve"),
m_absoluteEncoderOffset(absoluteEncoderOffset) {
    ctre::phoenix6::configs::TalonFXConfiguration speedConfigs{};
    ctre::phoenix6::configs::TalonFXConfiguration angleConfigs{};
    ctre::phoenix6::configs::TalonFXConfiguration encoderConfigs{};

    speedConfigs.Slot0.kP = SwerveDriveConstants::kPSpeed;
    speedConfigs.Slot0.kI = SwerveDriveConstants::kISpeed;
    speedConfigs.Slot0.kD = SwerveDriveConstants::kDSpeed;
    speedConfigs.Slot0.kV = SwerveDriveConstants::kVSpeed;

    m_speedMotor.GetConfigurator().Apply(speedConfigs);

    m_speedMotor.SetInverted(speedMotorInverted);

    angleConfigs.Slot0.kP = SwerveDriveConstants::kPAngle;
    angleConfigs.Slot0.kI = SwerveDriveConstants::kIAngle;
    angleConfigs.Slot0.kD = SwerveDriveConstants::kDAngle;
    angleConfigs.Slot0.kV = SwerveDriveConstants::kVAngle;

    angleConfigs.Feedback.FeedbackRemoteSensorID = absoluteEncoderID;
    angleConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue(SwerveDriveConstants::kFusedCANcoder);
    angleConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    m_angleMotor.GetConfigurator().Apply(angleConfigs);

    encoderConfigs.
    m_speedMotor.
    // Non-continuous feedback -- one rotation forward is not the same as zero rotations (distance has been traveled)
    m_speedMotor.ConfigFeedbackNotContinuous(true);
    m_speedMotor.SetNeutralMode(NeutralMode::Brake);
    m_speedMotor.Config_kP(0, speedP, 50);
    m_speedMotor.Config_kI(0, speedI, 50);
    m_speedMotor.Config_kD(0, speedD, 50);
    m_speedMotor.ConfigClosedloopRamp(0);
    m_speedMotor.ConfigOpenloopRamp(0.01);
    // Enable current limiting, set current to limit down to as ampLimit, current to start limiting to as the same, and
    // time before limiting to .1 second (arbitrary)
    m_speedMotor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration{
                                            true, SwerveDriveConstants::ampLimit, SwerveDriveConstants::ampLimit + 5.0, 0.1});
    // Baby Mode: Limiting the speed
    // Delete for comp:
    // m_speedMotor.ConfigPeakOutputForward(0.5);
    // m_speedMotor.ConfigPeakOutputReverse(-0.5);

    m_angleMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    // Allows PIDController to treat two angles as the same point on a circle
    m_angleMotor.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Unsigned_0_to_360);
    m_angleMotor.ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    m_angleMotor.SetNeutralMode(NeutralMode::Brake);
    m_angleMotor.Config_kP(0, angleP, 50);
    m_angleMotor.Config_kI(0, angleI, 50);
    m_angleMotor.Config_kD(0, angleD, 50);
    m_angleMotor.ConfigFeedbackNotContinuous(false);
    // Limits acceleration of motors and current drawn
    m_angleMotor.ConfigOpenloopRamp(SwerveDriveConstants::loopRampRate);
    // Enable current limiting, set current to limit down to as ampLimit, current to start limiting to as the same, and
    // time before limiting to .1 second (arbitrary)
    m_angleMotor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration{
                                            true, SwerveDriveConstants::ampLimit, SwerveDriveConstants::ampLimit + 5.0, 0.1});

    m_absoluteEncoder.ConfigSensorDirection(true);
    // Boots to absolute and reads encoder offsets, so the wheels do not need to be straight
    // When starting up code or even power cycling the robot
    m_absoluteEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
    m_absoluteEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    ResetSpeedEncoder();
    // Resets angle motors with absolute encoder offsets
    ResetAngleToAbsolute();
}

void SwerveModule::Stop() {
    m_speedMotor.StopMotor();
    m_angleMotor.StopMotor();
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &state) {
    const auto state = frc::SwerveModuleState::Optimize(state, frc::Rotation2d{units::degree_t{m_absoluteEncoder.GetPosition()}});
}