// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Not my captain
#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/SwerveDrive.h"


#include <string>
#include <vector>

#include "Constants.h"

class TestAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 TestAuto> {
 public:
  TestAuto(std::string filePath, SwerveDrive *swerve);

  private:
SwerveDrive *m_swerve;
};
