// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include "Constants.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>

class PoseEstimatorHelper : public frc2::SubsystemBase {
 public:
  PoseEstimatorHelper();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetPoseEstimator(frc::SwerveDrivePoseEstimator<4> *poseEstimator); 
  void UpdatePoseEstimator(wpi::array<frc::SwerveModulePosition, 4U> modulePositions, frc::Rotation2d rotation);
  frc::Pose2d GetEstimatedPose();
  void AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp, wpi::array<double, 3> visionMeasurementStdDevs);
  void ResetPose(frc::Rotation2d rotation, wpi::array<frc::SwerveModulePosition, 4> modulePositions, frc::Pose2d pose);

  private:
  frc::SwerveDrivePoseEstimator<4> *m_poseEstimator;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
