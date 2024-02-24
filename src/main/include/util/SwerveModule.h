// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <iostream>
#include <string>
#include "Constants.h"





struct PIDValues {
  double driveP;
  double driveI;
  double driveD;
  double angleP;
  double angleI;
  double angleD;
};

class SwerveModule {
 public:
  SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
               int CANcoderID, double CANcoderOffset);
  void ConfigDriveMotor();
  void ConfigAngleMotor(int CANcoderID);
  void ConfigCANcoder();
  void Stop();
  void UpdatePosition();
  void SetDesiredState(const frc::SwerveModuleState &state);
  frc::SwerveModulePosition GetPosition(bool refresh);
  frc::SwerveModuleState GetState(bool refresh);
  units::degree_t GetMotorAngle();
  units::degree_t GetEncoderAngle();
  units::meters_per_second_t GetDriveSpeed();
  units::volt_t GetDriveVoltage();
  units::volt_t GetAngleVoltage();
  std::vector<ctre::phoenix6::BaseStatusSignal*> GetSignals();
  void SetDriveVoltage(units::volt_t voltage);
  void SetAngleVoltage(units::volt_t voltage);
  void ResetDriveEncoder();
  frc::SwerveModuleState OptimizeAngle(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle);
  double NormalizeTo0To360(double currentAngle, double targetAngle);
  std::pair<ctre::phoenix6::hardware::TalonFX*, ctre::phoenix6::hardware::TalonFX*> GetMotorsForMusic();
  void UpdatePreferences();

 private:
  // Components
  ctre::phoenix6::hardware::TalonFX m_driveMotor;
  ctre::phoenix6::hardware::TalonFX m_angleMotor;
  ctre::phoenix6::hardware::CANcoder m_CANcoder;
  PIDValues m_PIDValues;

  int m_moduleNumber;
  double m_CANcoderOffset;
  double m_lastAngle;
  frc::SwerveModulePosition m_position{0_m, frc::Rotation2d{}};

  ctre::phoenix6::configs::TalonFXConfiguration m_driveConfigs{};
  ctre::phoenix6::configs::TalonFXConfiguration m_angleConfigs{};
  ctre::phoenix6::configs::CANcoderConfiguration m_encoderConfigs{};

  // Signals to hold motor sensor reports
  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_drivePosition = m_driveMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_anglePosition = m_angleMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_driveVelocity = m_driveMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_angleVelocity = m_angleMotor.GetVelocity();

  ctre::phoenix6::controls::VelocityVoltage m_driveSetter{0.0_rad / 1.0_s};
  ctre::phoenix6::controls::PositionVoltage m_angleSetter{0.0_rad};

  // String keys for PID preferences
  std::string m_drivePKey;
  std::string m_driveIKey;
  std::string m_driveDKey;
  std::string m_anglePKey;
  std::string m_angleIKey;
  std::string m_angleDKey;

  std::vector<ctre::phoenix6::BaseStatusSignal*> m_signals;
};