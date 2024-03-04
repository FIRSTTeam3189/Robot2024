// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TwoNoteAuto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoNoteAuto::TwoNoteAuto(SwerveDrive *swerve, Intake *intake, Shooter *shooter, StartingPosition position) :
m_swerve(swerve),
m_intake(intake),
m_shooter(shooter),
m_position(position),
m_rotationPIDController(SwerveDriveConstants::kPRot, SwerveDriveConstants::kIRot, SwerveDriveConstants::kDRot) {
  (void)AutoConstants::kAutonomousPaths[0];
  (void)VisionConstants::kSyncBytes[0];

  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    frc2::InstantCommand([this]{
      switch(m_position) {
        case(StartingPosition::BlueAmp) :
          m_swerve->SetPose(frc::Pose2d{0.65_m, 6.23_m, frc::Rotation2d{60.0_deg}}, false);
          break;
        case(StartingPosition::BlueMid) :
          m_swerve->SetPose(frc::Pose2d{1.3_m, 5.52_m, frc::Rotation2d{0.0_deg}}, false);
          break;
        case(StartingPosition::BlueSource) :
          m_swerve->SetPose(frc::Pose2d{0.65_m, 4.46_m, frc::Rotation2d{-60.0_deg}}, false);
          break;
        case(StartingPosition::RedAmp) :
          m_swerve->SetPose(frc::Pose2d{15.91_m, 6.23_m, frc::Rotation2d{120.0_deg}}, false);
          break;
        case(StartingPosition::RedMid) :
          m_swerve->SetPose(frc::Pose2d{15.25_m, 5.52_m, frc::Rotation2d{180.0_deg}}, false);
          break;
        case(StartingPosition::RedSource) :
          m_swerve->SetPose(frc::Pose2d{15.88_m, 4.46_m, frc::Rotation2d{-120.0_deg}}, false);
          break;
        default:
          break;
      }
    },{m_swerve}),
    frc2::WaitCommand(5.0_s),
    frc2::ParallelDeadlineGroup(
    frc2::WaitCommand(4.0_s),
    frc2::RunCommand([this]{
      units::angular_velocity::radians_per_second_t rot = 
            -units::angular_velocity::radians_per_second_t{
            m_rotationPIDController.Calculate(m_swerve->GetNormalizedYaw().value(), 180.0)
            * SwerveDriveConstants::kMaxAngularVelocity};
      m_swerve->Drive(1.0_mps, 0.225_mps, rot, true, frc::Translation2d{0.0_m, frc::Rotation2d{}});
    },{m_swerve}))
  //   SetShooterRotation(m_shooter, ShooterState::AutoScore),
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(1.5_s),
  //     RunShooter(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootExtendTarget)
  //   ),
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(ShooterConstants::kShootTime),
  //     RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
  //   ),
  //   frc2::InstantCommand([this]{
  //     m_shooter->SetExtension(0.0);
  //     m_shooter->SetRollerPower(0.0);
  //     m_shooter->SetLoaderPower(0.0);
  //   },{m_shooter}),
  //   frc2::ParallelCommandGroup(
  //     frc2::ParallelRaceGroup(
  //       frc2::WaitCommand(1.0_s),
  //       SetIntakeRotation(m_intake, IntakeState::Extended)
  //     ),
  //     frc2::ParallelRaceGroup(
  //       frc2::WaitCommand(1.0_s),
  //       SetShooterRotation(m_shooter, ShooterState::Load)
  //     )
  //   ),
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(1.5_s),
  //     frc2::RunCommand([this]{
  //       units::angular_velocity::radians_per_second_t rot = 
  //             -units::angular_velocity::radians_per_second_t{
  //             m_rotationPIDController.Calculate(m_swerve->GetNormalizedYaw().value(), 0.0)
  //             * SwerveDriveConstants::kMaxAngularVelocity};

  //       m_swerve->Drive(-1.0_mps, 0.225_mps, rot, true, frc::Translation2d{0.0_m, frc::Rotation2d{}});
  //     },{m_swerve}),
  //     RunLoader(m_shoo -- ter, ShooterConstants::kLoadPower, 0.0, ShooterEndCondition::None),
  //     RunIntake(m_intake, IntakeConstants::kIntakePower)
  //   ),
  //   frc2::InstantCommand([this]{
  //     m_intake->SetRollerPower(0.0);
  //     m_shooter->SetRollerPower(0.0);
  //     m_shooter->SetLoaderPower(0.0);
  //   },{m_intake, m_shooter}),
  //   frc2::ParallelCommandGroup(
  //     SetShooterRotation(m_shooter, ShooterState::Retracted),
  //     SetIntakeRotation(m_intake, IntakeState::Retracted)
  //   ),
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(1.5_s),
  //     frc2::RunCommand([this]{
  //       units::angular_velocity::radians_per_second_t rot = 
  //             -units::angular_velocity::radians_per_second_t{
  //             m_rotationPIDController.Calculate(m_swerve->GetNormalizedYaw().value(), 0.0)
  //             * SwerveDriveConstants::kMaxAngularVelocity};

  //       m_swerve->Drive(1.1_mps, -0.225_mps, rot, true, frc::Translation2d{0.0_m, frc::Rotation2d{}});
  //     },{m_swerve})
  //   ),
  //   SetShooterRotation(m_shooter, ShooterState::AutoScore),
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(1.5_s),
  //     RunShooter(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootExtendTarget)
  //   ),
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(ShooterConstants::kShootTime),
  //     RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
  //   ),
  //   frc2::InstantCommand([this]{
  //     m_shooter->SetExtension(0.0);
  //     m_shooter->SetRollerPower(0.0);
  //     m_shooter->SetLoaderPower(0.0);
  //   },{m_shooter})
  );
}
