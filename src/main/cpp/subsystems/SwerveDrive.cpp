// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveDrive.h"

SwerveDrive::SwerveDrive(PoseEstimatorHelper *helper):
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
m_moduleArray(
    &m_modules.m_frontLeft,
    &m_modules.m_frontRight,
    &m_modules.m_backLeft,
    &m_modules.m_backRight

    //addresses to access of each swerve module motor
),
m_pigeon(SwerveDriveConstants::kGyroID, "Swerve"),
m_poseHelper(helper),
m_modulePositions(
    m_modules.m_frontLeft.GetPosition(true),
    m_modules.m_frontRight.GetPosition(true),
    m_modules.m_backLeft.GetPosition(true),
    m_modules.m_backRight.GetPosition(true)
    //initializes the gyroscope and pose helpers for pose estimator
),

m_lastXSpeed(0.0_mps),
m_lastYSpeed(0.0_mps)

  {   
    (void)AutoConstants::kAutonomousPaths[0];
    (void)VisionConstants::kSyncBytes[0];
    ConfigGyro();
    ConfigSignals();
    auto poseEstimator = new frc::SwerveDrivePoseEstimator<4> (SwerveDriveConstants::kKinematics, m_pigeon.GetRotation2d(), 
    m_modulePositions, frc::Pose2d{}, VisionConstants::kEncoderTrustCoefficients, VisionConstants::kVisionTrustCoefficients);
    m_poseHelper->SetPoseEstimator(poseEstimator);

    // / Setup autobuilder for pathplannerlib
    pathplanner::AutoBuilder::configure(
        [this](){ return GetEstimatedAutoPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ SetPose(pose, false); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ DriveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            pathplanner::PIDConstants(AutoConstants::kPTranslationAuto, AutoConstants::kITranslationAuto, AutoConstants::kDTranslationAuto), // Translation PID constants
            pathplanner::PIDConstants(AutoConstants::kPRotationAuto, AutoConstants::kIRotationAuto, AutoConstants::kDRotationAuto) // Rotation PID constants
        ),
        m_autoRobotConfig,
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

    frc::Preferences::SetBoolean(m_tuningModeKey, false);
    frc::Preferences::SetBoolean(m_diagnosticsKey, true);
    
    m_drivePKey = "DriveP";
    m_driveIKey = "DriveI";
    m_driveDKey = "DriveD";
    m_anglePKey = "AngleP";
    m_angleIKey = "AngleI";
    m_angleDKey = "AngleD";
    m_rotationSKey = "RotS";

    frc::Preferences::SetDouble(m_drivePKey, SwerveModuleConstants::kPDrive);
    frc::Preferences::SetDouble(m_driveIKey, SwerveModuleConstants::kIDrive);
    frc::Preferences::SetDouble(m_driveDKey, SwerveModuleConstants::kDDrive);
    frc::Preferences::SetDouble(m_anglePKey, SwerveModuleConstants::kPAngle);
    frc::Preferences::SetDouble(m_angleIKey, SwerveModuleConstants::kIAngle);
    frc::Preferences::SetDouble(m_angleDKey, SwerveModuleConstants::kDAngle);
    frc::Preferences::SetDouble(m_rotationSKey, SwerveDriveConstants::kSRot);
}

// This method will be called once per scheduler run
void SwerveDrive::Periodic() {
    if (frc::Preferences::GetBoolean(m_tuningModeKey, false)) {
        m_modules.m_frontLeft.UpdatePreferences();
        m_modules.m_frontRight.UpdatePreferences();
        m_modules.m_backLeft.UpdatePreferences();
        m_modules.m_backRight.UpdatePreferences();

        // Update rotation S value
        m_rotationS = frc::Preferences::GetDouble(m_rotationSKey, SwerveDriveConstants::kSRot);
    }
    UpdateEstimator();
    RefreshAllSignals();
}

void SwerveDrive::RefreshAllSignals() {
    frc::SmartDashboard::PutString("Swerve signal status", ctre::phoenix6::BaseStatusSignal::WaitForAll(0.02_s, m_allSignals).GetName());
}

void SwerveDrive::ConfigSignals() {
    // Takes in robot frequency Talon FX using 
    for(int i=0; i < 4; i++) {
        auto signals = m_moduleArray[i]->GetSignals();
        for(int j=0; j < 4; j++) {
            m_allSignals.emplace_back(signals[j]);
        }
    }

    //get signals from swerve modules

    // frc::SmartDashboard::PutNumber("Signal refresh rate 0", m_allSignals[0]->GetAppliedUpdateFrequency().value());
    // frc::SmartDashboard::PutNumber("Signal refresh rate 1", m_allSignals[1]->GetAppliedUpdateFrequency().value());
    // frc::SmartDashboard::PutNumber("Signal refresh rate 2", m_allSignals[2]->GetAppliedUpdateFrequency().value());
    // frc::SmartDashboard::PutNumber("Signal refresh rate 3", m_allSignals[3]->GetAppliedUpdateFrequency().value());

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(SwerveDriveConstants::kRefreshRate, m_allSignals);
    //updates the frequency for all the signals
}

void SwerveDrive::ConfigGyro() {
    m_pigeon.GetConfigurator().Apply(ctre::phoenix6::configs::Pigeon2Configuration{});
    m_pigeonConfigs.MountPose.MountPoseYaw = SwerveDriveConstants::kGyroMountPoseYaw;
    m_pigeon.GetConfigurator().Apply(m_pigeonConfigs);
}

void SwerveDrive::Drive(units::meters_per_second_t xSpeed, 
                        units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                        bool fieldRelative,
                        frc::Translation2d centerOfRotation,
                        bool shouldDecelerate) {

    // Add static ff value depending on direction of travel (rot)
    if (rot > units::radians_per_second_t(0.0)){
        rot += units::radians_per_second_t(m_rotationS);
    } else if (rot < units::radians_per_second_t(0.0)) {
        rot -= units::radians_per_second_t(m_rotationS);
    }

    auto [xSpeedLimited, ySpeedLimited] = LimitDeceleration(xSpeed, ySpeed);

    if (shouldDecelerate){
        xSpeed = xSpeedLimited;
        ySpeed = ySpeedLimited;
    } 

    auto states = SwerveDriveConstants::kKinematics.ToSwerveModuleStates(
                  (fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_pigeon.GetRotation2d())
                            : frc::ChassisSpeeds{xSpeed, ySpeed, rot}),
                            centerOfRotation);

    
    auto [fl, fr, bl, br] = states;

    if (m_slowMode) {
        SwerveDriveConstants::kKinematics.DesaturateWheelSpeeds(&states, SwerveModuleConstants::kMaxSpeed * SwerveDriveConstants::kSlowModeDriveMultiplier);
    } else {
        SwerveDriveConstants::kKinematics.DesaturateWheelSpeeds(&states, SwerveModuleConstants::kMaxSpeed);
    }

    double AdvantageScopeDesiredStates[] = 
    {(double)fl.angle.Degrees(), (double)fl.speed,
     (double)fr.angle.Degrees(), (double)fr.speed,
     (double)bl.angle.Degrees(), (double)bl.speed,
     (double)br.angle.Degrees(), (double)br.speed};

    frc::SmartDashboard::PutNumberArray("AdvantageScope Desired States", AdvantageScopeDesiredStates);

    SetModuleStates(states);
}

// Takes in the old speeds and applies the slew rate limiter, returning the new limited x and y speeds
// Uses kinematics to take limited magnitude and reconstruct them into x and y speeds.
wpi::array<units::meters_per_second_t, 2> SwerveDrive::LimitDeceleration(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed){
    
    units::radian_t theta = units::radian_t{atan2(ySpeed.value(), xSpeed.value())};
    // If xSpeed and ySpeed (both) are 0, retrieve the last speeds and setting theta to it.
    // If xSpeed or ySpeed is greater than 0, update the last speeds to use in case input goes to 0 on the next cycle
    if (abs(xSpeed.value()) < SwerveDriveConstants::kDecelerationDeadband && abs(ySpeed.value()) < SwerveDriveConstants::kDecelerationDeadband){
        theta = units::radian_t{atan2(m_lastYSpeed.value(), m_lastXSpeed.value())};
        frc::SmartDashboard::PutNumber("Deceleration/IsUsingLastSpeed", true);
    } 
    else {
        frc::SmartDashboard::PutNumber("Deceleration/IsUsingLastSpeed", false);
        m_lastXSpeed = xSpeed;
        m_lastYSpeed = ySpeed;
    }
    

    // Get the magnitude of speeds using pythagorean theorem
    units::meters_per_second_t velMagnitude = units::meters_per_second_t{
                                                sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2))};\

    // Pass magnitude into limiter, returning a limited magnitude value
    units::meters_per_second_t velMagnitudeLimited = units::meters_per_second_t{
                                                        m_decelerationLimiter.Calculate(velMagnitude.value()).value()};

    // Reconstruct x and y speeds from limited magnitude
    units::meters_per_second_t xSpeedLimited = units::meters_per_second_t{
                                                    velMagnitudeLimited.value() * cos(theta.value())};
    units::meters_per_second_t ySpeedLimited = units::meters_per_second_t{
                                                    velMagnitudeLimited.value() * sin(theta.value())};

    frc::SmartDashboard::PutNumber("Deceleration/theta", theta.value());
    frc::SmartDashboard::PutNumber("Deceleration/velocity magnitude", velMagnitude.value());
    frc::SmartDashboard::PutNumber("Deceleration/velocity magnitude limited", velMagnitudeLimited.value());
    frc::SmartDashboard::PutNumber("Deceleration/x speed", xSpeed.value());
    frc::SmartDashboard::PutNumber("Deceleration/y speed", ySpeed.value());
    frc::SmartDashboard::PutNumber("Deceleration/x speed limited", xSpeedLimited.value());
    frc::SmartDashboard::PutNumber("Deceleration/y speed limited", ySpeedLimited.value());

    return wpi::array<units::meters_per_second_t, 2> {xSpeedLimited, ySpeedLimited};
}

void SwerveDrive::DriveRobotRelative(frc::ChassisSpeeds speeds) {
    // speeds.vx = -speeds.vx;
    // speeds.vy = -speeds.vy;

    // Add static ff value depending on direction of travel speeds omega
    if (speeds.omega > units::radians_per_second_t(0.0)){
        speeds.omega += units::radians_per_second_t(m_rotationS);
    } else if (speeds.omega < units::radians_per_second_t(0.0)) {
        speeds.omega -= units::radians_per_second_t(m_rotationS);
    }

    auto states = SwerveDriveConstants::kKinematics.ToSwerveModuleStates(speeds);
    auto [fl, fr, bl, br] = states;
    SwerveDriveConstants::kKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxAutoModuleSpeed);

    double AutoDesiredStates[] = 
    {(double)fl.angle.Degrees(), (double)fl.speed,
     (double)fr.angle.Degrees(), (double)fr.speed,
     (double)bl.angle.Degrees(), (double)bl.speed,
     (double)br.angle.Degrees(), (double)br.speed};

    frc::SmartDashboard::PutNumberArray("Auto Desired States", AutoDesiredStates);
    frc::SmartDashboard::PutNumber("Robot relative desired x speed", speeds.vx.value());
    frc::SmartDashboard::PutNumber("Robot relative desired y speed", speeds.vy.value());
    frc::SmartDashboard::PutNumber("Robot relative desired omega speed", speeds.omega.value());

    SetModuleStates(states);
}

void SwerveDrive::ToggleSlowMode() {
    m_slowMode = !m_slowMode;
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

    auto speeds = SwerveDriveConstants::kKinematics.ToChassisSpeeds(
        frontLeftModuleState, frontRightModuleState, backLeftModuleState, backRightModuleState);

    frc::SmartDashboard::PutNumber("Robot relative x speed", speeds.vx.value());
    frc::SmartDashboard::PutNumber("Robot relative y speed", speeds.vy.value());
    frc::SmartDashboard::PutNumber("Robot relative omega speed", speeds.omega.value());
    return speeds;
}

//uses odometry to get teh speeds of the chassis

frc::Pose2d SwerveDrive::GetEstimatedPose() {
    UpdateEstimator();
    return m_poseHelper->GetEstimatedPose();
}

frc::Pose2d SwerveDrive::GetEstimatedAutoPose() {
    UpdateEstimator();
    auto pose = m_poseHelper->GetEstimatedPose();
    // pose = frc::Pose2d(pose.X(), pose.Y(), -pose.Rotation());
    frc::SmartDashboard::PutNumber("Auto pose x", pose.X().value());
    frc::SmartDashboard::PutNumber("Auto pose y", pose.Y().value());
    frc::SmartDashboard::PutNumber("Auto pose rot", pose.Rotation().Degrees().value());
    return pose;
}

void SwerveDrive::LogModuleStates(wpi::array<frc::SwerveModulePosition, 4> modulePositions) {
    double AdvantageScopeMeasuredStates[] = 
    {modulePositions[0].angle.Degrees().value(), m_moduleArray[0]->GetDriveSpeed().value(),
     modulePositions[1].angle.Degrees().value(), m_moduleArray[1]->GetDriveSpeed().value(),
     modulePositions[2].angle.Degrees().value(), m_moduleArray[2]->GetDriveSpeed().value(),
     modulePositions[3].angle.Degrees().value(), m_moduleArray[3]->GetDriveSpeed().value()};
  frc::SmartDashboard::PutNumberArray("AdvantageScope Measured States", AdvantageScopeMeasuredStates);
}

void SwerveDrive::UpdateEstimator() {
    // Get current positions and update
    m_modulePositions[0] = m_modules.m_frontLeft.GetPosition(true);
    m_modulePositions[1] = m_modules.m_frontRight.GetPosition(true);
    m_modulePositions[2] = m_modules.m_backLeft.GetPosition(true);
    m_modulePositions[3] = m_modules.m_backRight.GetPosition(true);
    m_poseHelper->UpdatePoseEstimator(m_modulePositions, frc::Rotation2d(GetNormalizedYaw()));
    LogModuleStates(m_modulePositions);
}

void SwerveDrive::ResetDriveEncoders() {
    m_modules.m_frontLeft.ResetDriveEncoder();
    m_modules.m_frontRight.ResetDriveEncoder();
    m_modules.m_backLeft.ResetDriveEncoder();
    m_modules.m_backRight.ResetDriveEncoder();
}

void SwerveDrive::SetPose(frc::Pose2d pose, bool justRotation) {
    UpdateEstimator();
    if (justRotation) {
        auto currentPose = GetEstimatedPose();
        m_poseHelper->ResetPose(GetNormalizedYaw(), m_modulePositions, frc::Pose2d{currentPose.X(), currentPose.Y(), pose.Rotation()});
        // m_poseHelper->ResetPose(pose.Rotation(), m_modulePositions, frc::Pose2d{currentPose.X(), currentPose.Y(), pose.Rotation()});
    }
    else {
        m_poseHelper->ResetPose(GetNormalizedYaw(), m_modulePositions, pose);
        // m_poseHelper->ResetPose(pose.Rotation(), m_modulePositions, pose);
    }
    //sets pose to current pose
    frc::SmartDashboard::PutNumber("Auto starting pose x", pose.X().value());
    frc::SmartDashboard::PutNumber("Auto starting pose y", pose.Y().value());
    frc::SmartDashboard::PutNumber("Auto starting pose rot", pose.Rotation().Degrees().value());
    for (int i = 0; i < 10; i++) {
        m_pigeon.SetYaw(pose.Rotation().Degrees());
    }
}

void SwerveDrive::ResetGyroscope() {
    m_pigeon.SetYaw(0.0_deg);
    // to correct yaw positioning
}

void SwerveDrive::Lock() {
    m_modules.m_frontLeft.SetDesiredState(frc::SwerveModuleState{0.0_mps, units::degree_t{45.0}});
    m_modules.m_frontRight.SetDesiredState(frc::SwerveModuleState{0.0_mps, units::degree_t{-45.0}});
    m_modules.m_backLeft.SetDesiredState(frc::SwerveModuleState{0.0_mps, units::degree_t{-45.0}});
    m_modules.m_backRight.SetDesiredState(frc::SwerveModuleState{0.0_mps, units::degree_t{45.0}});
    //holds at that position
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
    //gets velocity vector from the x and y speeds
    return units::meters_per_second_t{velocity};
}

void SwerveDrive::SetBrakeMode(BrakeMode mode) {
    for (auto mod: m_moduleArray) {
        mod->SetBrakeMode(mode);
    }
}
