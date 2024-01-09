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
#include <math.h>
#include <iostream>

class SwerveModule {
 public:
  SwerveModule(int speedMotorID, int angleMotorID, bool speedMotorInverted, bool angleMotorInverted,
               int absoluteEncoderID, double absoluteEncoderOffset, bool absoluteEncoderInverted);
  void Stop();
  void SetDesiredState(const frc::SwerveModuleState &state);
  frc::SwerveModuleState GetState();
  frc::SwerveModuleState OptimizeAngle(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle);
  double NormalizeTo0To360(double currentAngle, double targetAngle);
  double GetAbsoluteAngle();
  double GetRelativeAngle();
  double GetVelocity();
  void ResetSpeedEncoder();
  void ResetAngleToAbsolute();

 private:
  ctre::phoenix6::hardware::TalonFX m_speedMotor;
  ctre::phoenix6::hardware::TalonFX m_angleMotor;
  ctre::phoenix6::hardware::CANcoder m_absoluteEncoder;

  double m_absoluteEncoderOffset;
  double m_lastAbsoluteEncoderAngle;
  ctre::phoenix6::StatusSignal<double> m_speedPosition;
  ctre::phoenix6::StatusSignal<double> m_angleVelocity;
  ctre::phoenix6::StatusSignal<double> m_speedPosition;
  ctre::phoenix6::StatusSignal<double> m_angleVelocity;
};
