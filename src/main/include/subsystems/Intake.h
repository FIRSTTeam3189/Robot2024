// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <frc/controller/PIDController.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/RelativeEncoder.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  void SetPower(double rollerPower, double extensionPower);
  void Extend(double position);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
 
 private:
 rev::CANSparkMax m_extensionMotor;
 rev::CANSparkMax m_rollerMotor;
 rev::SparkMaxPIDController m_extensionPIDController;
 rev::RelativeEncoder m_extensionEncoder;
 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
