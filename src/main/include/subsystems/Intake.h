// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  void SetPower(double rollerPower, double extensionPower);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 rev::CANSparkFlex m_extensionMotor;
 rev::CANSparkMax m_rollerMotor;

 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
