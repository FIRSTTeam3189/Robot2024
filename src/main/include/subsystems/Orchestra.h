// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/Orchestra.hpp>
#include <frc/Filesystem.h>

class Orchestra : public frc2::SubsystemBase {
 public:
  Orchestra(std::array<ctre::phoenix6::hardware::TalonFX*, 8> motors);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::Orchestra m_orchestra;
};