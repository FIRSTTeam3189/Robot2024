// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Transform3d.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/FloatTopic.h>

#include "subsystems/PoseEstimatorHelper.h"
#include "Constants.h"
#include <iostream>
#include <span>

struct VisionData {
  bool isDetected;
  int ID;
  double lastTimestamp;
  std::vector<float> translationMatrix{0.0f, 0.0f, 0.0f};
  std::vector<float> rotationMatrix{0.0f, 0.0f, 0.0f};
};

class Vision : public frc2::SubsystemBase {
 public:
  Vision(PoseEstimatorHelper *helper); 
  VisionData GetVisionData();
  frc::Pose3d TagToCamera();
  frc::Pose3d CameraToRobot(frc::Pose3d cameraPose);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
 private:
  PoseEstimatorHelper *m_helper;
  nt::IntegerTopic m_tagIDTopic;
  nt::BooleanTopic m_isDetectedTopic;
  nt::FloatTopic m_lastTimestampTopic;
  nt::FloatArrayTopic m_translationMatrixTopic;
  nt::FloatArrayTopic m_rotationMatrixTopic;
  VisionData m_data;
  frc::Transform3d m_cameraToRobotTransform;
};
