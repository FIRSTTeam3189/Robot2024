// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/pch.h"
#include "subsystems/Orchestra.h"

Orchestra::Orchestra(std::array<ctre::phoenix6::hardware::TalonFX*, 8> motors) :
m_orchestra(motors, frc::filesystem::GetDeployDirectory() + "/music/funkytown.chrp") {
    m_orchestra.Play();
}

// This method will be called once per scheduler run
void Orchestra::Periodic() {}