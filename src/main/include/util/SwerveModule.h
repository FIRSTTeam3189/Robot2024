// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX>

class SwerveModule {
 public:
  SwerveModule();

 private:
  WPI_TalonFX speedMotor;
  WPI_TalonFX angleMotor;
};
