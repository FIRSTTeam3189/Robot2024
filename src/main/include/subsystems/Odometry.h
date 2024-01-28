// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/Pigeon2.hpp> 
#include <frc/geometry/Pose2d.h>
#include "Constants.h"
#include "Vision.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>

class Odometry : public frc2::SubsystemBase {
 public:
  Odometry(Vision *vision);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  ctre::phoenix6::hardware::Pigeon2 *GetPigeon();
  ctre::phoenix6::configs::Pigeon2Configuration *GetConfig();
  void SetPoseEstimator(frc::SwerveDrivePoseEstimator<4> *poseEstimator ); 
  frc::SwerveDrivePoseEstimator<4> *GetPoseEstimator();
  void UpdateOdometry(wpi::array<frc::SwerveModulePosition, 4U> modulePositions);
  private:
  ctre::phoenix6::hardware::Pigeon2 m_pigeon; 
  frc::SwerveDrivePoseEstimator<4> *m_poseEstimator;
  Vision *m_vision;
  ctre::phoenix6::configs::Pigeon2Configuration m_pigeonConfigs{};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
