// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MobilityAuto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
MobilityAuto::MobilityAuto(SwerveDrive *swerve) 
: m_swerve(swerve) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    frc2::InstantCommand([this]{
      m_swerve->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}}, true);
    },{m_swerve}),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(2.5_s),
      frc2::RunCommand([this]{
        m_swerve->Drive(1.0_mps, 0.0_mps, units::radians_per_second_t{0.0}, false, frc::Translation2d{0.0_m, frc::Rotation2d{}});
      },{m_swerve})
    )
  );
}
