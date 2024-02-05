// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Timer.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkAbsoluteEncoder.h> 
#include <rev/CANSparkMax.h>
#include "Constants.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  void SetRollerPower(double power);
  void SetRotationPower(double power);
  void SetRotation(units::degree_t target);
  units::degree_t GetRotation();
  bool NoteDetected();
  void UpdateUltrasonic();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
 
 private:
  rev::CANSparkMax m_rotationMotor;
  rev::CANSparkMax m_rollerMotor;
  frc::TrapezoidProfile<units::degrees>::Constraints m_constraints;
  frc::ProfiledPIDController<units::degrees> m_rotationPIDController;
  frc::ArmFeedforward m_ff;
  rev::SparkMaxAbsoluteEncoder m_rotationEncoder;
  units::degree_t m_target;
  frc::AnalogPotentiometer m_ultrasonicSensor;
  bool m_noteDetected;
  units::degrees_per_second_t m_lastSpeed;
  units::second_t m_lastTime;
 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
