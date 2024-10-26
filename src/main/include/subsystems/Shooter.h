// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include <frc/DigitalInput.h>
#include <frc/Preferences.h>
#include <frc/RobotController.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkFlex.h>
#include <rev/SparkAbsoluteEncoder.h> 
#include <rev/SparkRelativeEncoder.h> 
#include "Constants/ShooterConstants.h"
#include "Constants/GlobalConstants.h"
#include "util/ShooterAlignUtil.h"
#include "subsystems/PoseEstimatorHelper.h"

enum class ShooterState { None, Retracted, Load, DirectLoad, Close, Mid, Far, Zero, AutoAlign, StartingConfig, AutoScore, StartingAuto, ArbitraryAngle, InterpolateAngle } ;
enum class ShooterEndCondition { None, EndOnFirstDetection };

class Shooter : public frc2::SubsystemBase {
 public:
 
  Shooter(PoseEstimatorHelper *estimator);
  void SetRollerPower(double power);
  void SetRotationPower(double power);
  void SetLoaderPower(double power);
  void SetRotation(units::degree_t target);
  units::degree_t GetRotation();
  void ConfigRollerMotor();
  void ConfigLoaderMotor();
  void ConfigRotationMotor();
  void ConfigPID();
  void SetBrakeMode(BrakeMode mode);
  void UpdatePreferences();
  void SetState(ShooterState state, units::degree_t autoAlignAngle = ShooterConstants::kRetractTarget);
  units::degree_t GetTarget();
  void HoldPosition(units::degree_t target);
  void SetActive(bool active);
  bool NoteDetected();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
   // rev::CANSparkFlex m_topRollerMotor;
   // rev::CANSparkFlex m_bottomRollerMotor;
   rev::CANSparkMax m_topRollerMotor;
   rev::CANSparkMax m_bottomRollerMotor;
   rev::SparkRelativeEncoder m_rollerEncoder;
   rev::CANSparkMax m_loaderMotor;
   rev::CANSparkMax m_rotationMotor;

   frc::DigitalInput m_limitSwitchLeft;
   frc::DigitalInput m_limitSwitchRight;
   
   ShooterAlignUtil m_alignUtil;
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
};
