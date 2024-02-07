// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveDrive.h"

SwerveDrive::SwerveDrive(PoseEstimatorHelper *helper) :
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
m_pigeon(SwerveDriveConstants::kGyroID, "Swerve"),
m_poseHelper(helper),
m_modulePositions(
    m_modules.m_frontLeft.GetPosition(true),
    m_modules.m_frontRight.GetPosition(true),
    m_modules.m_backLeft.GetPosition(true),
    m_modules.m_backRight.GetPosition(true)
)
{   
    ConfigGyro();
    auto poseEstimator = new frc::SwerveDrivePoseEstimator<4> (SwerveDriveConstants::kKinematics, m_pigeon.GetRotation2d(), 
    m_modulePositions, frc::Pose2d{}, VisionConstants::kEncoderTrustCoefficients, VisionConstants::kVisionTrustCoefficients);
    m_poseHelper->SetPoseEstimator(poseEstimator);

    // / Setup autobuilder for pathplannerlib
    pathplanner::AutoBuilder::configureHolonomic(
        [this](){ return GetEstimatedPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ SetPose(pose, false); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ DriveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        AutoConstants::autoConfig,
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

// This method will be called once per scheduler run
void SwerveDrive::Periodic() {
    if (frc::Preferences::GetBoolean(m_tuningModeKey, false)) {
        m_modules.m_frontLeft.UpdatePreferences();
        m_modules.m_frontRight.UpdatePreferences();
        m_modules.m_backLeft.UpdatePreferences();
        m_modules.m_backRight.UpdatePreferences();
    }
    UpdateEstimator();
}

void SwerveDrive::ConfigGyro() {
    m_pigeon.GetConfigurator().Apply(ctre::phoenix6::configs::Pigeon2Configuration{});
    m_pigeonConfigs.MountPose.MountPoseYaw = SwerveDriveConstants::kGyroMountPoseYaw;
    m_pigeon.GetConfigurator().Apply(m_pigeonConfigs);
}

void SwerveDrive::Drive(units::meters_per_second_t xSpeed, 
                        units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                        bool fieldRelative,
                        frc::Translation2d centerOfRotation) {

    frc::SmartDashboard::PutNumber("X speed 2", xSpeed.value());
    frc::SmartDashboard::PutNumber("Y speed 2", ySpeed.value());
    frc::SmartDashboard::PutNumber("Rot 2", rot.value());

    auto states = SwerveDriveConstants::kKinematics.ToSwerveModuleStates(
                  (fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                    xSpeed, ySpeed, rot,  m_pigeon.GetRotation2d())
                                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot}),
                  centerOfRotation);

    auto [fl, fr, bl, br] = states;

    double AdvantageScopeDesiredStates[] = 
    {(double)fl.angle.Degrees(), (double)fl.speed,
     (double)fr.angle.Degrees(), (double)fr.speed,
     (double)bl.angle.Degrees(), (double)bl.speed,
     (double)br.angle.Degrees(), (double)br.speed};

    frc::SmartDashboard::PutNumberArray("AdvantageScope Desired States", AdvantageScopeDesiredStates);

    SwerveDriveConstants::kKinematics.DesaturateWheelSpeeds(&states, SwerveDriveConstants::kMaxSpeed);
    SetModuleStates(states);
}

void SwerveDrive::DriveRobotRelative(frc::ChassisSpeeds speeds) {
    auto states = SwerveDriveConstants::kKinematics.ToSwerveModuleStates(speeds);
    SwerveDriveConstants::kKinematics.DesaturateWheelSpeeds(&states, SwerveDriveConstants::kMaxSpeed);
    m_modules.m_frontLeft.SetDesiredState(states[0]);
    m_modules.m_frontRight.SetDesiredState(states[1]);
    m_modules.m_backLeft.SetDesiredState(states[2]);
    m_modules.m_backRight.SetDesiredState(states[3]);
}

void SwerveDrive::SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates) {
    m_modules.m_frontLeft.SetDesiredState(desiredStates[0]);
    m_modules.m_frontRight.SetDesiredState(desiredStates[1]);
    m_modules.m_backLeft.SetDesiredState(desiredStates[2]);
    m_modules.m_backRight.SetDesiredState(desiredStates[3]);
}

units::degree_t SwerveDrive::GetNormalizedYaw() {
    // Normalizes yaw to (-180, 180)
    double yaw =  m_pigeon.GetYaw().GetValue().value(); 
    // (-360, 360)
    double normalizedYaw = fmod(yaw, 360.0);
    // (-180, 180)
    if (normalizedYaw > 180)
        normalizedYaw -= 360;
    else if (normalizedYaw < -180)
        normalizedYaw += 360;

    frc::SmartDashboard::PutNumber("Normalized Yaw", normalizedYaw);
    return units::degree_t{normalizedYaw};
}

void SwerveDrive::SetRobotYaw(double yaw){
    m_pigeon.SetYaw(units::degree_t{yaw});
}

frc::ChassisSpeeds SwerveDrive::GetRobotRelativeSpeeds() {
    frc::SwerveModuleState frontLeftModuleState = m_modules.m_frontLeft.GetState(true);
    frc::SwerveModuleState frontRightModuleState = m_modules.m_frontRight.GetState(true);
    frc::SwerveModuleState backLeftModuleState = m_modules.m_backLeft.GetState(true);
    frc::SwerveModuleState backRightModuleState = m_modules.m_backRight.GetState(true);

    return SwerveDriveConstants::kKinematics.ToChassisSpeeds(
        frontLeftModuleState, frontRightModuleState, backLeftModuleState, backRightModuleState);
}

frc::Pose2d SwerveDrive::GetEstimatedPose() {
    UpdateEstimator();
    return m_poseHelper->GetEstimatedPose();
}

void SwerveDrive::LogModuleStates(wpi::array<frc::SwerveModulePosition, 4> modulePositions) {
    double AdvantageScopeMeasuredStates[] = 
    {modulePositions[0].angle.Degrees().value(), modulePositions[0].distance.value(),
     modulePositions[1].angle.Degrees().value(), modulePositions[1].distance.value(),
     modulePositions[2].angle.Degrees().value(), modulePositions[2].distance.value(),
     modulePositions[3].angle.Degrees().value(), modulePositions[3].distance.value()};
  frc::SmartDashboard::PutNumberArray("AdvantageScope Measured States", AdvantageScopeMeasuredStates);
}

void SwerveDrive::UpdateEstimator() {
    // Get current positions and update
    m_modulePositions[0] = m_modules.m_frontLeft.GetPosition(true);
    m_modulePositions[1] = m_modules.m_frontRight.GetPosition(true);
    m_modulePositions[2] = m_modules.m_backLeft.GetPosition(true);
    m_modulePositions[3] = m_modules.m_backRight.GetPosition(true);

    LogModuleStates(m_modulePositions);
    m_poseHelper->UpdatePoseEstimator(m_modulePositions, frc::Rotation2d{GetNormalizedYaw()});
}

void SwerveDrive::ResetDriveEncoders() {
    m_modules.m_frontLeft.ResetDriveEncoder();
    m_modules.m_frontRight.ResetDriveEncoder();
    m_modules.m_backLeft.ResetDriveEncoder();
    m_modules.m_backRight.ResetDriveEncoder();
}

void SwerveDrive::SetPose(frc::Pose2d pose, bool justRotation) {
    UpdateEstimator();
    // TODO: not sure if the encoders should be reset here
    // ResetDriveEncoders();
    if (justRotation) {
        auto currentPose = GetEstimatedPose();
        m_poseHelper->ResetPose(GetNormalizedYaw(), m_modulePositions, frc::Pose2d{currentPose.X(), currentPose.Y(), pose.Rotation()});
    }
    else
        m_poseHelper->ResetPose(GetNormalizedYaw(), m_modulePositions, pose);
}

void SwerveDrive::ResetGyroscope() {
    m_pigeon.SetYaw(0.0_deg);
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

std::array<ctre::phoenix6::hardware::TalonFX*, 8> SwerveDrive::GetMotorsForMusic() {
    auto frontLeftMotors = m_modules.m_frontLeft.GetMotorsForMusic();
    auto frontRightMotors = m_modules.m_frontLeft.GetMotorsForMusic();
    auto backLeftMotors = m_modules.m_frontLeft.GetMotorsForMusic();
    auto backRightMotors = m_modules.m_frontLeft.GetMotorsForMusic();

    return std::array{
        frontLeftMotors.first, frontLeftMotors.second,
        frontRightMotors.first, frontRightMotors.second,
        backLeftMotors.first, backLeftMotors.second,
        backRightMotors.first, backRightMotors.second
    };
}

units::meters_per_second_t SwerveDrive::GetTotalVelocity() {
    auto chassisSpeeds = GetRobotRelativeSpeeds();
    auto x = chassisSpeeds.vx.value();
    auto y = chassisSpeeds.vy.value();
    double velocity = sqrt((x * x) + (y * y));
    return units::meters_per_second_t{velocity};
}