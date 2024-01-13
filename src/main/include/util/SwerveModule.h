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
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "Constants.h"

struct Signals {
  ctre::phoenix6::StatusSignal<units::angle::turn_t> drivePosition;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> anglePosition;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> driveVelocity;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> angleVelocity;
};

class SwerveModule {
 public:
  SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
               int CANcoderID, double CANcoderOffset);
  void ConfigDriveMotor();
  void ConfigAngleMotor(int CANcoderID);
  void ConfigCANcoder();
  void Stop();
  void SetDesiredState(const frc::SwerveModuleState &state);
  void UpdatePosition();
  frc::SwerveModulePosition GetPosition(bool refresh);
  frc::SwerveModuleState GetState(bool refresh);
  units::degree_t GetMotorAngle();
  units::degree_t GetEncoderAngle();
  units::meters_per_second_t GetDriveSpeed();
  Signals GetSignals();

 private:
  ctre::phoenix6::hardware::TalonFX m_driveMotor;
  ctre::phoenix6::hardware::TalonFX m_angleMotor;
  ctre::phoenix6::hardware::CANcoder m_CANcoder;

  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_drivePosition = m_driveMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_anglePosition = m_angleMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_driveVelocity = m_driveMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_angleVelocity = m_angleMotor.GetVelocity();
  Signals m_signals;

  int m_moduleNumber;
  double m_CANcoderOffset;
  frc::SwerveModulePosition m_position{0_m, frc::Rotation2d{}};

  ctre::phoenix6::controls::VelocityVoltage m_driveSetter{0.0_rad / 1.0_s};
  ctre::phoenix6::controls::PositionVoltage m_angleSetter{0.0_rad};
};