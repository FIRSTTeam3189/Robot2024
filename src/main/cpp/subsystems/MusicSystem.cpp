// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//plays song

#include "subsystems/MusicSystem.h"

MusicSystem::MusicSystem(std::array<ctre::phoenix6::hardware::TalonFX*, 8> motors) :
m_musicSystem(std::string{frc::filesystem::GetDeployDirectory() + "/music/funkytown.chrp"}.c_str()) {
    // for (auto instrument: motors) {
        // m_musicSystem.AddInstrument(*instrument);
    // }
    // m_musicSystem.Play();
    // std::cout << "Music constructing\n";
}

// This method will be called once per scheduler run
void MusicSystem::Periodic() {}