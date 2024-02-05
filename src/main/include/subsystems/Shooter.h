// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h> 
#include "Constants.h"

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  void SetRollerPower(double power);
  void SetRotationPower(double power);
  void SetExtensionPower(double power);
  void SetLoaderPower(double power);
  void SetRotation(units::degree_t target);
  void SetExtension(double target);
  void StopRotation();
  units::degree_t GetRotation();
  double GetExtension();
  void ConfigRollerMotor();
  void ConfigExtensionMotor();
  void ConfigRotationMotor();
  bool NoteDetected();
  void UpdateUltrasonic();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
   rev::CANSparkMax m_rollerMotor;
   rev::CANSparkMax m_loaderMotor;
   rev::CANSparkMax m_extensionMotor;
   rev::CANSparkMax m_rotationMotor;
   frc::TrapezoidProfile<units::degrees>::Constraints m_constraints;
   frc::ProfiledPIDController<units::degrees> m_rotationPIDController;
   frc::ArmFeedforward m_ff;
   rev::SparkMaxPIDController m_extensionPIDController;
   rev::SparkMaxAbsoluteEncoder m_rotationEncoder;
   rev::SparkMaxAbsoluteEncoder m_extensionEncoder;
   frc::AnalogPotentiometer m_ultrasonicSensor;
   bool m_noteDetected;
   units::degree_t m_target;
   units::degrees_per_second_t m_lastSpeed;
   units::second_t m_lastTime;
};