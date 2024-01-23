// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <rev/CANSparkMax.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  void SetTopPower(double power);
  void SetBottomPower(double power);
  void SetAnglePower(double power);
  void SetExtension(double position);
   
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
   rev::CANSparkMax m_topMotor;
   rev::CANSparkMax m_bottomMotor;
   rev::CANSparkMax m_extensionMotor;
   rev::SparkMaxPIDController m_extensionPIDController;
   rev::SparkMaxAlternateEncoder m_extensionEncoder;
};