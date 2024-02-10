// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/SwerveModule.h"

SwerveModule::SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
                           int CANcoderID, double CANcoderOffset):
m_driveMotor(driveMotorID, "Swerve"),
m_angleMotor(angleMotorID, "Swerve"),
m_CANcoder(CANcoderID, "Swerve"),
m_PIDValues{SwerveModuleConstants::kPDrive, SwerveModuleConstants::kIDrive, SwerveModuleConstants::kDDrive,
            SwerveModuleConstants::kPAngle, SwerveModuleConstants::kIAngle, SwerveModuleConstants::kDAngle},
m_moduleNumber(moduleNumber),
m_CANcoderOffset(CANcoderOffset),
m_signals{m_drivePosition, m_anglePosition, m_driveVelocity, m_angleVelocity}
 {
    ConfigDriveMotor();
    ConfigAngleMotor(CANcoderID);
    ConfigCANcoder();
    
    // Setup preferences class, which allows editing values while robot is enabled
    // Very useful for PID tuning
    m_drivePKey = "DriveP" + std::to_string(m_moduleNumber);
    m_driveIKey = "DriveI" + std::to_string(m_moduleNumber);
    m_driveDKey = "DriveD" + std::to_string(m_moduleNumber);
    m_anglePKey = "AngleP" + std::to_string(m_moduleNumber);
    m_angleIKey = "AngleI" + std::to_string(m_moduleNumber);
    m_angleDKey = "AngleD" + std::to_string(m_moduleNumber);

    frc::Preferences::InitDouble(m_drivePKey, m_driveConfigs.Slot0.kP);
    frc::Preferences::InitDouble(m_driveIKey, m_driveConfigs.Slot0.kI);
    frc::Preferences::InitDouble(m_driveDKey, m_driveConfigs.Slot0.kD);
    frc::Preferences::InitDouble(m_anglePKey, m_driveConfigs.Slot0.kP);
    frc::Preferences::InitDouble(m_angleIKey, m_driveConfigs.Slot0.kI);
    frc::Preferences::InitDouble(m_angleDKey, m_driveConfigs.Slot0.kD);
}

void SwerveModule::ConfigDriveMotor() {
    // Set to factory default
    m_driveMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

    m_driveConfigs.Slot0.kP = m_PIDValues.driveP;
    m_driveConfigs.Slot0.kI = m_PIDValues.driveI;
    m_driveConfigs.Slot0.kD = m_PIDValues.driveD;
    m_driveConfigs.Slot0.kV = SwerveModuleConstants::kVDrive;
    m_driveConfigs.Slot0.kS = SwerveModuleConstants::kSDrive;

    m_driveConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants::kDriveContinuousCurrentLimit;
    m_driveConfigs.CurrentLimits.SupplyCurrentThreshold = SwerveModuleConstants::kDrivePeakCurrentLimit;
    m_driveConfigs.CurrentLimits.SupplyTimeThreshold = SwerveModuleConstants::kDrivePeakCurrentDuration;
    m_driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = SwerveModuleConstants::kDriveEnableCurrentLimit;

    m_driveConfigs.MotorOutput.Inverted = SwerveModuleConstants::kDriveMotorInverted;

    m_driveConfigs.MotorOutput.NeutralMode = SwerveModuleConstants::kDriveNeutralMode;
    
    m_driveConfigs.Feedback.SensorToMechanismRatio = SwerveModuleConstants::kDriveGearRatio;

    m_driveMotor.GetConfigurator().Apply(m_driveConfigs);

    m_driveMotor.SetPosition(0.0_rad);
}

void SwerveModule::ConfigAngleMotor(int CANcoderID) {
    // Set to factory default
    m_angleMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration({}));

    m_angleConfigs.Slot0.kP = m_PIDValues.angleP;
    m_angleConfigs.Slot0.kI = m_PIDValues.angleI;
    m_angleConfigs.Slot0.kD = m_PIDValues.angleD;
    m_angleConfigs.Slot0.kV = SwerveModuleConstants::kVAngle;
    m_angleConfigs.Slot0.kS = SwerveModuleConstants::kSAngle;

    m_angleConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    // m_angleConfigs.Feedback.SensorToMechanismRatio = SwerveModuleConstants::kAngleGearRatio;
    // TODO: Not sure if this number is correct/if it actually works this way on our modules
    m_angleConfigs.Feedback.RotorToSensorRatio = SwerveModuleConstants::kAngleGearRatio;
    m_angleConfigs.Feedback.FeedbackRemoteSensorID = CANcoderID;
    m_angleConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
    // m_angleConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;

    m_angleConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants::kAngleContinuousCurrentLimit;
    m_angleConfigs.CurrentLimits.SupplyCurrentThreshold = SwerveModuleConstants::kAnglePeakCurrentLimit;
    m_angleConfigs.CurrentLimits.SupplyTimeThreshold = SwerveModuleConstants::kAnglePeakCurrentDuration;
    m_angleConfigs.CurrentLimits.SupplyCurrentLimitEnable = SwerveModuleConstants::kAngleEnableCurrentLimit;

    m_angleConfigs.Voltage.PeakForwardVoltage = SwerveModuleConstants::kMaxVoltage;
    m_angleConfigs.Voltage.PeakReverseVoltage = -SwerveModuleConstants::kMaxVoltage;

    m_angleConfigs.MotorOutput.Inverted = SwerveModuleConstants::kAngleMotorInverted;
    m_angleConfigs.MotorOutput.NeutralMode = SwerveModuleConstants::kAngleNeutralMode;

    m_angleMotor.GetConfigurator().Apply(m_angleConfigs);
}

void SwerveModule::ConfigCANcoder() {
    // Set to factory default
    m_CANcoder.GetConfigurator().Apply(ctre::phoenix6::configs::CANcoderConfiguration{});

    // Set the magnet offset in configs
    m_encoderConfigs.MagnetSensor.MagnetOffset = m_CANcoderOffset;
    m_encoderConfigs.MagnetSensor.SensorDirection = SwerveModuleConstants::kCANcoderInverted;
    m_encoderConfigs.MagnetSensor.AbsoluteSensorRange = SwerveModuleConstants::kCANcoderSensorRange;

    m_CANcoder.GetConfigurator().Apply(m_encoderConfigs);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &state) {
    const auto optimizedState = frc::SwerveModuleState::Optimize(state, m_position.angle);
    // double targetSpeed = optimizedState.speed.value() * SwerveModuleConstants::kRotationsPerMeter;
    // auto targetAngle = optimizedState.angle.Degrees() / 360.0;
    // auto optimizedState = OptimizeAngle(state, m_position.angle);
    double targetSpeed = optimizedState.speed.value() * SwerveModuleConstants::kRotationsPerMeter;
    auto targetAngle = optimizedState.angle.Degrees();

    std::string speedKey = std::to_string(m_moduleNumber) + " target speed";
    std::string angleKey = std::to_string(m_moduleNumber) + " target angle";
    frc::SmartDashboard::PutNumber(speedKey, targetSpeed);
    frc::SmartDashboard::PutNumber(angleKey, targetAngle.value());

    // m_driveMotor.SetControl(m_driveSetter.WithVelocity(units::turns_per_second_t{targetSpeed}));
    // FOC Pro feature
    m_driveMotor.SetControl(m_driveSetter.WithEnableFOC(true).WithVelocity(units::turns_per_second_t{targetSpeed}));
    if (fabs(targetSpeed) < .05 && fabs(m_lastAngle - targetAngle.value()) < 5.0) {
        // Stop();
        m_driveMotor.SetControl(m_driveSetter.WithEnableFOC(true).WithVelocity(units::turns_per_second_t{0.0}));
        targetAngle = units::degree_t{m_lastAngle};
    } else {
        m_angleMotor.SetControl(m_angleSetter.WithEnableFOC(true).WithPosition(targetAngle));
    }
    
    m_lastAngle = targetAngle.value();
}

frc::SwerveModuleState SwerveModule::OptimizeAngle(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle) {
    // Optimizes the module to take shortest turning path so it never rotates more than 180 degrees
    double targetAngle = NormalizeTo0To360(
                        currentAngle.Degrees().value(), 
                        desiredState.angle.Degrees().value());

    auto targetSpeed = desiredState.speed;
    double delta = targetAngle - currentAngle.Degrees().value();
    frc::SmartDashboard::PutNumber("Delta", delta);
    // If degrees to target is > 90, reverse the speed motor direction
    if (std::abs(delta) > 90) {
        targetSpeed = -targetSpeed;
        // If difference is positive, subtract 180 deg and reverse; if negative, vice versa
        if (delta > 90) {
            targetAngle -= 180;
        } else {
            targetAngle += 180;
        }
        // targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    auto targetAngleRad = units::radian_t{targetAngle / SwerveModuleConstants::kDEGToRAD};
    return frc::SwerveModuleState{targetSpeed, targetAngleRad};
}

double SwerveModule::NormalizeTo0To360(double currentAngle, double targetAngle) {
    // 0 to 360 range we want to be in
    double lowerBound;
    double upperBound;
    // Offset from current angle to lower bound
    int lowerOffset = (int)currentAngle % 360;
    // frc::SmartDashboard::PutNumber("Normalize Lower Offset", lowerOffset);
    /* If angle is positive, angle will be set from 0 to 360, else -360 to 0
    * These degree numbers are relative to input angle
    * This is because WPILib optimize expects a continuous input from a PIDController
    * But Falcon500s use internal PIDControllers without continuous input
    */ 
    if (lowerOffset >= 0) {
          lowerBound = currentAngle - lowerOffset;
          upperBound = currentAngle + (360 - lowerOffset);
      } else {
          upperBound = currentAngle - lowerOffset;
          lowerBound = currentAngle - (360 + lowerOffset);
      }
    frc::SmartDashboard::PutNumber("Normalize Lower Bound", lowerBound);
    frc::SmartDashboard::PutNumber("Normalize Upper Bound", upperBound);
      // Target angle is now normalized between either 0 to 360 or -360 to 0
      while (targetAngle < lowerBound) {
          targetAngle += 360;      }
      while (targetAngle > upperBound) {
          targetAngle -= 360;
      }
      if (targetAngle - currentAngle > 180) {
          targetAngle -= 360;
      } else if (targetAngle - currentAngle < -180) {
          targetAngle += 360;
      }
      return targetAngle;
}

Signals SwerveModule::GetSignals() {
    return m_signals;
}

void SwerveModule::Stop() {
    m_driveMotor.SetControl(m_driveSetter.WithVelocity(units::turns_per_second_t{0.0}));
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

void SwerveModule::ResetDriveEncoder() {
    m_driveMotor.SetPosition(units::turn_t{0.0});
}

void SwerveModule::UpdatePreferences() {
    m_driveConfigs.Slot0.kP = frc::Preferences::GetDouble(m_drivePKey, SwerveModuleConstants::kPDrive);
    m_driveConfigs.Slot0.kI = frc::Preferences::GetDouble(m_driveIKey, SwerveModuleConstants::kIDrive);
    m_driveConfigs.Slot0.kD = frc::Preferences::GetDouble(m_driveDKey, SwerveModuleConstants::kDDrive);
    m_angleConfigs.Slot0.kP = frc::Preferences::GetDouble(m_anglePKey, SwerveModuleConstants::kPAngle);
    m_angleConfigs.Slot0.kI = frc::Preferences::GetDouble(m_angleIKey, SwerveModuleConstants::kIAngle);
    m_angleConfigs.Slot0.kD = frc::Preferences::GetDouble(m_angleDKey, SwerveModuleConstants::kDAngle);

    m_driveMotor.GetConfigurator().Apply(m_driveConfigs);
    m_angleMotor.GetConfigurator().Apply(m_angleConfigs);
    // m_CANcoder.GetConfigurator().Apply(m_encoderConfigs);
}

std::pair<ctre::phoenix6::hardware::TalonFX*, ctre::phoenix6::hardware::TalonFX*> SwerveModule::GetMotorsForMusic() {
    return std::pair{&m_driveMotor, &m_angleMotor};
}

units::volt_t SwerveModule::GetDriveVoltage() {
    return m_driveMotor.GetMotorVoltage().GetValue();
}

units::volt_t SwerveModule::GetAngleVoltage() {
    return m_angleMotor.GetMotorVoltage().GetValue();
}

void SwerveModule::SetAngleVoltage(units::volt_t voltage) {
    m_angleMotor.SetVoltage(voltage);
}

void SwerveModule::SetDriveVoltage(units::volt_t voltage) {
    m_driveMotor.SetVoltage(voltage);
}