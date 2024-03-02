// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TwoNoteAuto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoNoteAuto::TwoNoteAuto(SwerveDrive *swerve, Intake *intake, Shooter *shooter) :
m_swerve(swerve),
m_intake(intake),
m_shooter(shooter) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    SetShooterRotation(m_shooter, ShooterState::Close),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(1.5_s),
      RunShooter(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootExtendTarget),
    ),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    ),
    frc2::InstantCommand([this]{
      m_shooter->SetExtension(0.0);
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}),
    frc2::ParallelCommandGroup(
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetIntakeRotation(m_intake, IntakeState::Extended)
      ),
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetShooterRotation(m_shooter, ShooterState::Load)
      )
    ),
    frc2::ParallelCommandGroup(
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(2.0_s),
        frc2::RunCommand([this]{
          m_swerve->Drive(1.5_mps, 0.0_mps, units::radians_per_second_t{0.0}, false, frc::Translation2d{0.0_m, frc::Rotation2d{}});
        },{m_swerve})
      ),
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0, ShooterEndCondition::None),
      RunIntake(m_intake, IntakeConstants::kIntakePower)
    ),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_intake, m_shooter}),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Retracted),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    ),

    SetShooterRotation(m_shooter, ShooterState::DirectLoad),
    // RunLoader(m_shooter, ShooterConstants::kDirectLoadPower, ShooterConstants::kDirectLoadPower, ShooterEndCondition::EndOnFirstDetection),
    RunLoader(m_shooter, ShooterConstants::kDirectLoadPower, ShooterConstants::kDirectLoadPower, ShooterEndCondition::None),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Retracted),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    ),
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Retracted),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    ),

    SetShooterRotation(m_shooter, ShooterState::Close),
    RunShooter(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootExtendTarget),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    ),
    frc2::InstantCommand([this]{
      m_shooter->SetExtension(0.0);
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    })
    ));
}
