// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkAbsoluteEncoder.h> 
#include "Constants/ClimberConstants.h" 
#include "Constants/GlobalConstants.h"

class Climber : public frc2::SubsystemBase {
 public:
  Climber();
  void SetPower(double leftPower, double rightPower);
  void SetBrakeMode(BrakeMode mode);
  void ToggleGlobalBrakeMode();
  BrakeMode GetBrakeMode();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::CANSparkMax m_leftMotor;
  rev::CANSparkMax m_rightMotor;
  rev::SparkRelativeEncoder m_leftEncoder;
  rev::SparkRelativeEncoder m_rightEncoder;

  frc::DigitalInput m_brakeModeLimitSwitch;
  BrakeMode m_brakeMode;
  bool m_lastLimitSwitchDetectionState;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
