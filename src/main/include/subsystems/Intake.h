// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/SparkAbsoluteEncoder.h> 
#include "Constants.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  void SetPower(double rollerPower, double rotationPower);
  void SetRotation(double position);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
 
 private:
 rev::CANSparkMax m_rotationMotor;
 rev::CANSparkMax m_rollerMotor;
 rev::SparkMaxPIDController m_rotationPIDController;
 rev::SparkMaxAbsoluteEncoder m_rotationEncoder;
 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
