// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Timer.h>
#include <frc/DigitalInput.h>
#include <frc/Preferences.h>
#include <frc/RobotController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkAbsoluteEncoder.h> 
#include <rev/CANSparkMax.h>
#include "Constants/IntakeConstants.h"
#include "Constants/GlobalConstants.h"

enum class IntakeState { None, Extended, Amp, Retracted };

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  void SetRollerPower(double power);
  void SetRotationPower(double power);
  void SetRotation(units::degree_t target);
  units::degree_t GetRotation();
  units::degree_t GetTarget();
  void SetState(IntakeState state);
  void UpdatePreferences();
  void ConfigRollerMotor();
  void ConfigRotationMotor();
  void SetBrakeMode(BrakeMode mode);
  void HoldPosition();
  void SetActive(bool active);
  bool NoteDetected();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 protected:
  void ConfigPID();

 private:
  rev::CANSparkMax m_rotationMotor;
  rev::CANSparkMax m_rollerMotor;
  frc::DigitalInput m_limitSwitchLeft;
  frc::DigitalInput m_limitSwitchRight;
  frc::TrapezoidProfile<units::degrees>::Constraints m_constraints;
  frc::ProfiledPIDController<units::degrees> m_profiledPIDController;
  frc::ArmFeedforward *m_ff;
  rev::SparkMaxAbsoluteEncoder m_rotationEncoder;
  units::degree_t m_target;
  units::degrees_per_second_t m_lastSpeed;
  units::degrees_per_second_t m_lastTargetSpeed;
  units::degrees_per_second_squared_t m_acceleration;
  units::degrees_per_second_squared_t m_targetAcceleration;
  units::second_t m_lastTime;
  bool m_isActive;
  
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
};
