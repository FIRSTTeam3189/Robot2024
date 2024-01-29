// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h> 
#include "Constants.h" 
#include "frc/Servo.h" 

class Climber : public frc2::SubsystemBase {
 public:
  Climber();
  void SetPower(double power);
  void SetServoRotation(double angle);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::CANSparkMax m_leftMotor;
  rev::CANSparkMax m_rightMotor;
  frc::Servo m_leftServo;
  frc::Servo m_rightServo;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
