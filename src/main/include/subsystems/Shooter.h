// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogPotentiometer.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h> 
#include "Constants.h"

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  void SetRollerPower(double power);
  void SetAnglePower(double power);
  void SetRotation(double position);
  void SetExtension(double position);
  void ConfigRollerMotor();
  void ConfigExtensionMotors();
  void ConfigRotationMotor();
  void SetLoaderPower(double power);
   
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
   rev::CANSparkMax m_topMotor;
   rev::CANSparkMax m_bottomMotor;
   rev::CANSparkMax m_loaderMotor;
   rev::CANSparkMax m_leftExtensionMotor;
   rev::CANSparkMax m_rightExtensionMotor;
   rev::CANSparkMax m_rotationMotor;
   rev::SparkMaxPIDController m_rotationPIDController;
   rev::SparkMaxPIDController m_leftExtensionPIDController;
   rev::SparkMaxPIDController m_rightExtensionPIDController;
   rev::SparkMaxAbsoluteEncoder m_rotationEncoder;
   rev::SparkMaxAbsoluteEncoder m_leftExtensionEncoder;
   rev::SparkMaxAbsoluteEncoder m_rightExtensionEncoder;
   frc::AnalogPotentiometer m_ultrasonicSensor;
};