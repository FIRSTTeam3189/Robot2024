// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Transform3d.h>

#include "wpinet/EventLoopRunner.h"
#include "wpinet/HttpServerConnection.h"
#include "wpinet/UrlParser.h"
#include "wpinet/uv/Loop.h"
#include "wpinet/uv/Tcp.h"

// #include <networktables/NetworkTable.h>
// #include <networktables/NetworkTableInstance.h>
// #include <networktables/FloatArrayTopic.h>
// #include <networktables/BooleanTopic.h>
// #include <networktables/IntegerTopic.h>
// #include <networktables/FloatTopic.h>

#include "subsystems/PoseEstimatorHelper.h"
#include "Constants.h"
#include <iostream>
#include <span>

// Pack struct tightly so the are no buffer bytes in between data members
#pragma pack(push, 1)

struct VisionData {
  uint8_t isDetected = 0;
  uint64_t ID = 0;
  double lastTimestamp = 0.0;
  // std::vector<float> translationMatrix{0.0f, 0.0f, 0.0f};
  // std::vector<float> rotationMatrix{0.0f, 0.0f, 0.0f};
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
  void SetupTCPConnection();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
 private:
  PoseEstimatorHelper *m_helper;
  // nt::IntegerTopic m_tagIDTopic;
  // nt::BooleanTopic m_isDetectedTopic;
  // nt::FloatTopic m_lastTimestampTopic;
  // nt::FloatArrayTopic m_translationMatrixTopic;
  // nt::FloatArrayTopic m_rotationMatrixTopic;
  VisionData m_data;
  frc::Transform3d m_cameraToRobotTransform;

  // TCP stuff
  std::shared_ptr<wpi::uv::Tcp> m_TCP;
};