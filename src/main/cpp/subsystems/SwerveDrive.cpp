// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveDrive.h"

SwerveDrive::SwerveDrive() :
m_modules{
  {+SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
  {+SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter},
  {-SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
  {-SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter},
  {0, SwerveModuleConstants::kFrontLeftDriveID, SwerveModuleConstants::kFrontLeftAngleID,
   SwerveModuleConstants::kFrontLeftCANcoderID, SwerveModuleConstants::kFrontLeftOffset},
  {1, SwerveModuleConstants::kFrontRightDriveID, SwerveModuleConstants::kFrontRightAngleID,
   SwerveModuleConstants::kFrontRightCANcoderID, SwerveModuleConstants::kFrontRightOffset},
  {2, SwerveModuleConstants::kBackLeftDriveID, SwerveModuleConstants::kBackLeftAngleID,
   SwerveModuleConstants::kBackLeftCANcoderID, SwerveModuleConstants::kBackLeftOffset},
  {3, SwerveModuleConstants::kBackRightDriveID, SwerveModuleConstants::kBackRightAngleID,
   SwerveModuleConstants::kBackRightCANcoderID, SwerveModuleConstants::kBackRightOffset}
},
m_pigeon(SwerveDriveConstants::kGyroID, "Swerve")
{

}

// This method will be called once per scheduler run
void SwerveDrive::Periodic() {
    if (frc::Preferences::GetBoolean(m_tuningModeKey, false)) {
        m_modules.m_frontLeft.UpdatePreferences();
        m_modules.m_frontRight.UpdatePreferences();
        m_modules.m_backLeft.UpdatePreferences();
        m_modules.m_backRight.UpdatePreferences();
    }
}

void SwerveDrive::Drive(units::meters_per_second_t xSpeed, 
                        units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                        bool fieldRelative,
                        frc::Translation2d centerOfRotation) {

    auto states = SwerveDriveConstants::kKinematics.ToSwerveModuleStates(
                  (fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                    xSpeed, ySpeed, rot, m_pigeon.GetRotation2d())
                                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot}),
                  centerOfRotation);

    SwerveDriveConstants::kKinematics.DesaturateWheelSpeeds(&states, SwerveDriveConstants::kMaxSpeed);
    SetModuleStates(states);
}

void SwerveDrive::SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates) {
    m_modules.m_frontLeft.SetDesiredState(desiredStates[0]);
    m_modules.m_frontRight.SetDesiredState(desiredStates[1]);
    m_modules.m_backLeft.SetDesiredState(desiredStates[2]);
    m_modules.m_backRight.SetDesiredState(desiredStates[3]);
}

double SwerveDrive::GetNormalizedYaw() {
    // Normalizes yaw to (-180, 180)
    int yaw = m_pigeon.GetYaw().GetValue().value(); 
    // (-360, 360)
    int normalizedYaw = (yaw % 360);
    // (-180, 180)
    if (normalizedYaw > 180)
        normalizedYaw -= 360;
    else if (normalizedYaw < -180)
        normalizedYaw += 360;

    frc::SmartDashboard::PutNumber("Normalized Yaw", normalizedYaw);
    return normalizedYaw;
}

void SwerveDrive::Lock() {
    m_modules.m_frontLeft.SetDesiredState(frc::SwerveModuleState{0.0_mps, units::degree_t{45.0}});
    m_modules.m_frontRight.SetDesiredState(frc::SwerveModuleState{0.0_mps, units::degree_t{-45.0}});
    m_modules.m_backLeft.SetDesiredState(frc::SwerveModuleState{0.0_mps, units::degree_t{-45.0}});
    m_modules.m_backRight.SetDesiredState(frc::SwerveModuleState{0.0_mps, units::degree_t{45.0}});
}

void SwerveDrive::Stop() {
    m_modules.m_frontLeft.Stop();
    m_modules.m_frontRight.Stop();
    m_modules.m_backLeft.Stop();
    m_modules.m_backRight.Stop();
}