// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Pose2d.h>
#include <frc/RobotController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/DriverStation.h>
#include <frc/filter/SlewRateLimiter.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include "util/SwerveModule.h"
#include "subsystems/PoseEstimatorHelper.h"
#include "Constants/SwerveDriveConstants.h"
#include "Constants/VisionConstants.h"
#include "Constants/AutoConstants.h"

enum class DriveState {HeadingControl, RotationVelocityControl, SpeakerAlign, SpeakerAlignTranslationAlgorithm, ArbitraryAngleAlign, SourceAlign } ;

struct SwerveModules {
  frc::Translation2d m_frontLeftLocation;
  frc::Translation2d m_frontRightLocation;
  frc::Translation2d m_backLeftLocation;
  frc::Translation2d m_backRightLocation;

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;
};

class SwerveDrive : public frc2::SubsystemBase {
 public:
  void RefreshAllSignals();
  void ConfigSignals();
  void UpdatePreferences();
  SwerveDrive(PoseEstimatorHelper *helper);
  void SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates);
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed,
             units::radians_per_second_t rot,
             bool fieldRelative,
             frc::Translation2d centerOfRotation);
  void Lock();
  void Stop();
  units::degree_t GetNormalizedYaw();
  void SetRobotYaw(double yaw);
  units::meters_per_second_t GetTotalVelocity();
  frc::ChassisSpeeds GetRobotRelativeSpeeds();
  wpi::array<units::meters_per_second_t, 2> LimitDeceleration(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed);
  void DriveRobotRelative(frc::ChassisSpeeds speeds);
  void ToggleSlowMode();
  frc::Pose2d GetEstimatedPose();
  frc::Pose2d GetEstimatedAutoPose();
  void SetPose(frc::Pose2d pose, bool justRotation);
  void ConfigGyro();
  void SetBrakeMode(BrakeMode mode);
  void ResetGyroscope();
  void ResetDriveEncoders();
  void ToggleTuningMode();
  void UpdateEstimator();
  void LogModuleStates(wpi::array<frc::SwerveModulePosition, 4> modulePositions);
  std::array<ctre::phoenix6::hardware::TalonFX*, 8> GetMotorsForMusic();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveModules m_modules;
  wpi::array<SwerveModule*, 4> m_moduleArray;
  ctre::phoenix6::hardware::Pigeon2 m_pigeon;
  PoseEstimatorHelper *m_poseHelper;
  wpi::array<frc::SwerveModulePosition, 4> m_modulePositions;
  ctre::phoenix6::configs::Pigeon2Configuration m_pigeonConfigs{};

  // Tuning mode preference -- when true, will constantly update module preferences
  std::string_view m_tuningModeKey = "Tuning Mode";
  std::string_view m_diagnosticsKey = "Full Diagnostics";
 
  bool m_slowMode = false;

  std::string m_drivePKey;
  std::string m_driveIKey;
  std::string m_driveDKey;
  std::string m_anglePKey;
  std::string m_angleIKey;
  std::string m_angleDKey;
  std::string m_rotationSKey;

  double m_rotationS = SwerveDriveConstants::kSRot;

  std::vector<ctre::phoenix6::BaseStatusSignal*> m_allSignals;

  // Limits the magnitude of the negative acceleration in the direction of the robot's travel
  // Adding large positive number to not limit the positive ROC, but adding actual negative to limit negative ROC
  frc::SlewRateLimiter<units::scalar> m_decelerationLimiter{100000 / 1_s, SwerveDriveConstants::kDecelerationLimit / 1_s};
};
