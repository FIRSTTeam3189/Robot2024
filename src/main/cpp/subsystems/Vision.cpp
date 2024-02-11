// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"
#include "wpinet/EventLoopRunner.h"
#include "wpinet/HttpServerConnection.h"
#include "wpinet/UrlParser.h"
#include "wpinet/uv/Loop.h"
#include "wpinet/uv/Tcp.h"

namespace uv = wpi::uv;

Vision::Vision(PoseEstimatorHelper *helper) : 
m_helper(helper),
m_data(), 
m_cameraToRobotTransform(VisionConstants::kCameraXOffset, VisionConstants::kCameraYOffset, VisionConstants::kCameraZOffset,
  frc::Rotation3d{0.0_rad, 0.0_rad, VisionConstants::kCameraYawOffset}) { 
//   nt::NetworkTableInstance networkTableInstance = nt::NetworkTableInstance::GetDefault();
//   m_isDetectedTopic = networkTableInstance.GetBooleanTopic("Vision/Detection");
//   m_tagIDTopic = networkTableInstance.GetIntegerTopic("Vision/AprilTag/ID");
//   m_lastTimestampTopic = networkTableInstance.GetFloatTopic("Vision/AprilTag/Timestamp");
//   m_translationMatrixTopic = networkTableInstance.GetFloatArrayTopic("Vision/AprilTag/TMatrix");
//   m_rotationMatrixTopic = networkTableInstance.GetFloatArrayTopic("Vision/AprilTag/RMatrix");

  // TCP stuff
  SetupTCPConnection();
//   m_TCP->Connect("10.31.89.59", 8010, [this]{});
}

// This method will be called once per scheduler run
void Vision::Periodic() {
    if (VisionConstants::kShouldUseVision) {
        // bool isDetected = m_isDetectedTopic.Subscribe(0).Get();

        if (m_data.isDetected) {
            // m_data.ID = m_tagIDTopic.Subscribe(-1).Get();
            // float defaultArray[]{0.0f, 0.0f, 0.0f};
            // std::span s{defaultArray, 3};

            // m_data.translationMatrix = m_translationMatrixTopic.Subscribe(defaultArray).Get();
            // m_data.rotationMatrix = m_rotationMatrixTopic.Subscribe(defaultArray).Get();
            // m_data.lastTimestamp = m_lastTimestampTopic.Subscribe(0.0).Get();

            // m_TCP->StartRead();

            frc::SmartDashboard::PutNumber("Vision X distance", m_data.translationMatrix[0]);
            frc::SmartDashboard::PutNumber("Vision Y distance", m_data.translationMatrix[1]);
            frc::SmartDashboard::PutNumber("Vision Z distance", m_data.translationMatrix[2]);

            // Turn distances into robot pose
            frc::Pose3d cameraPose = TagToCamera();
            frc::Pose3d robotPose = CameraToRobot(cameraPose);
            units::meter_t tagDistance = units::meter_t{sqrt(pow(m_data.translationMatrix[0], 2.0) 
                                                           + pow(m_data.translationMatrix[1], 2.0))};
            // Calculate vision std devs based on tag distance
            double stdDevDistanceCompensation = tagDistance.value() * VisionConstants::kVisionStdDevPerMeter;
            auto baseVisionStdDevs = VisionConstants::kVisionTrustCoefficients;
            auto distanceCompensatedStdDevs = wpi::array<double, 3>{
                                                baseVisionStdDevs[0] + stdDevDistanceCompensation,
                                                baseVisionStdDevs[1] + stdDevDistanceCompensation,
                                                baseVisionStdDevs[2] + stdDevDistanceCompensation};

            units::second_t timestamp = units::second_t{m_data.lastTimestamp};
            m_helper->AddVisionMeasurement(robotPose.ToPose2d(), timestamp, distanceCompensatedStdDevs);
        }
    }
}

frc::Pose3d Vision::TagToCamera() {
    frc::Pose3d tagPose = VisionConstants::kTagPoses.at(m_data.ID - 1);
    // Invert the data for x on tags to the right since vision reports positive differences and 
    // TransformBy adds so we need to subtract
    auto xDistance = (tagPose.X() - m_helper->GetEstimatedPose().X() > 0.0_m) ? -m_data.translationMatrix[0] : m_data.translationMatrix[0];
    frc::Transform3d tagToCamera = frc::Transform3d(
        units::meter_t{xDistance},
        units::meter_t{-m_data.translationMatrix[1]},
        units::meter_t{-m_data.translationMatrix[2]},
        frc::Rotation3d{
            // Just using the yaw rotation only
            0.0_deg,
            units::degree_t{-m_data.rotationMatrix[1]},
            0.0_deg
        }
    );

    return tagPose.TransformBy(tagToCamera);
}

frc::Pose3d Vision::CameraToRobot(frc::Pose3d cameraPose) {
    return cameraPose.TransformBy(m_cameraToRobotTransform);
}

VisionData Vision::GetVisionData() {
    return m_data;
}

void Vision::SetupTCPConnection() {
    wpi::EventLoopRunner loop;
    loop.ExecAsync([this](uv::Loop& loop) {
        m_TCP = uv::Tcp::Create(loop);

        // bind to listen address and port
        m_TCP->Bind("10.31.89.59", 8010);

        // when we get a connection, accept it and start reading
        m_TCP->connection.connect([srv = m_TCP.get(), this] {
            m_TCP = srv->Accept();
            if (!m_TCP) {
                return;
            }
            std::fputs("Got a connection\n", stderr);
        });

        // start listening for incoming connections
        m_TCP->Listen();

        std::fputs("Listening on port 8010\n", stderr);
    });

    m_TCP->data.connect_connection([this](uv::Buffer& buf, size_t size) {
        if (size == 0)
            return;
        auto byteSpan = buf.bytes();  
        std::vector<uint8_t> data;
        for (uint8_t Tascheel: byteSpan) {
            data.emplace_back(Tascheel);
        }
        void* rawData = data.data();
        m_data = *reinterpret_cast<VisionData*>(rawData);
    });

    m_TCP->StartRead();
}