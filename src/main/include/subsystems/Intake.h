// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/Preferences.h>
#include <frc/RobotController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMaxPIDController.h>
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
  units::degree_t GetTarget();
  void SetState(IntakeState state);
  bool NoteDetected();
  void UpdateUltrasonic();
  void UpdatePreferences();
  void ConfigRollerMotor();
  void ConfigRotationMotor();
  void SetSlowMode(bool slow);

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 protected:
  void ConfigPID();

 private:
  rev::CANSparkMax m_rotationMotor;
  rev::CANSparkMax m_rollerMotor;
  frc::TrapezoidProfile<units::degrees>::Constraints m_constraints;
  frc::ProfiledPIDController<units::degrees> m_profiledPIDController;
  rev::SparkMaxPIDController m_rotationPIDController;
  frc::ArmFeedforward *m_ff;
  rev::SparkMaxAbsoluteEncoder m_rotationEncoder;
  std::optional<units::degree_t> m_target;
  frc::AnalogPotentiometer m_ultrasonicSensor;
  bool m_noteDetected;
  units::degrees_per_second_t m_lastSpeed;
  units::degrees_per_second_t m_lastTargetSpeed;
  units::degrees_per_second_squared_t m_acceleration;
  units::degrees_per_second_squared_t m_targetAcceleration;
  units::second_t m_lastTime;
  frc2::sysid::SysIdRoutine m_sysIdRoutine;
  IntakeState m_currentState;
  IntakeState m_prevState;
  bool m_slow;
  int m_loopsSinceEnabled;
  
  // String keys for PID preferences
  std::string m_rotationPKey;
  std::string m_rotationIKey;
  std::string m_rotationDKey;
  std::string m_rotationGKey;
  std::string m_rotationSKey;
  std::string m_rotationVKey;
  std::string m_rotationAKey;
  std::string m_rotationTargetKey;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  std::map<IntakeState, std::array<double, 3>> kRotationTargetPID {
        {{IntakeState::Extended}, {IntakeConstants::kPRotation, IntakeConstants::kIRotation, IntakeConstants::kDRotation}},
        {{IntakeState::Amp}, {IntakeConstants::kPRotation, IntakeConstants::kIRotation, IntakeConstants::kDRotation}},
        {{IntakeState::Retracted}, {IntakeConstants::kPRotation, IntakeConstants::kIRotation, IntakeConstants::kDRotation}}
    };
};
