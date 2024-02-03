// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Autos/TestAuto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// Add your commands here, e.g.
// AddCommands(FooCommand{}, BarCommand{});

TestAuto::TestAuto(std::string filePath, SwerveDrive *swerve ) : m_swerve(swerve) {

  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.emplace_back(pathplanner::AutoBuilder::buildAuto(filePath).Unwrap());
  auto group = SequentialCommandGroup(std::move(commands));
  
  AddCommands(
    frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
    std::move(group)
  );

}
