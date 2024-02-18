// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Timer.h>
#include <frc/Preferences.h>
#include <frc/RobotController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/SparkAbsoluteEncoder.h> 
#include <rev/CANSparkMax.h>
#include "Constants.h"

enum class IntakeState { None, Extended, Amp, Retracted };

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  void SetRollerPower(double power);
  void SetRotationPower(double power);
  void SetRotation(units::degree_t target);
  units::degree_t GetRotation();
  bool NoteDetected();
  void UpdateUltrasonic();
  void UpdatePreferences();

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 protected:
  void Config();
  
 private:
  rev::CANSparkMax m_rotationMotor;
  rev::CANSparkMax m_rollerMotor;
  frc::TrapezoidProfile<units::degrees>::Constraints m_constraints;
  frc::ProfiledPIDController<units::degrees> m_rotationPIDController;
  // rev::SparkMaxPIDController m_rotationPIDController;
  frc::ArmFeedforward *m_ff;
  rev::SparkMaxAbsoluteEncoder m_rotationEncoder;
  units::degree_t m_target;
  frc::AnalogPotentiometer m_ultrasonicSensor;
  bool m_noteDetected;
  units::degrees_per_second_t m_lastSpeed;
  units::second_t m_lastTime;
  frc2::sysid::SysIdRoutine m_sysIdRoutine;
  IntakeState m_currentState;
  IntakeState m_prevState;
  
  // String keys for PID preferences
  std::string m_rotationPKey;
  std::string m_rotationIKey;
  std::string m_rotationDKey;
  std::string m_rotationGKey;
  std::string m_rotationSKey;
  std::string m_rotationVKey;
  std::string m_rotationAKey;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
