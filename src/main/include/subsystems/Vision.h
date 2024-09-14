// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Transform3d.h>
#include <frc/SerialPort.h>
#include <frc/RobotBase.h>
#include <frc/PowerDistribution.h>
#include <frc/Preferences.h>

#include "subsystems/PoseEstimatorHelper.h"
#include "Constants/VisionConstants.h"

#include <iostream>
#include <vector>
#include <optional>
#include <iomanip>
#include <sstream>

// Pack struct tightly so the are no buffer bytes in between data members
#pragma pack(push, 1)
struct VisionData {
  uint8_t isDetected = 0;
  uint64_t ID = 0;
  double lastTimestamp = 0.0;
  double translationMatrix[3] = {0.0, 0.0, 0.0};
  double rotationMatrix[3] = {0.0, 0.0, 0.0};
};
#pragma pack(pop)

class Vision : public frc2::SubsystemBase {
 public:
  Vision(PoseEstimatorHelper *helper); 
  VisionData GetVisionData();
  frc::Pose3d TagToCamera();
  frc::Pose3d CameraToRobot(frc::Pose3d cameraPose);
  void UpdatePosition();
  void UpdateData();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
 private:
  PoseEstimatorHelper *m_poseEstimator;
  VisionData m_data;
  frc::Transform3d m_cameraToRobotTransform;
  frc::SerialPort m_serialCam;
  std::vector<char> m_buffer;
  std::string m_visionEnabledKey;
  bool m_visionEnabled;

  // Private functions
  std::optional<VisionData> ParseData();
};