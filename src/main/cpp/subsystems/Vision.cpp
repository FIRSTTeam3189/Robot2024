// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision(PoseEstimatorHelper *helper) : 
m_helper(helper),
m_data(), 
m_cameraToRobotTransform(VisionConstants::kCameraXOffset, VisionConstants::kCameraYOffset, VisionConstants::kCameraZOffset,
  frc::Rotation3d{0.0_rad, 0.0_rad, VisionConstants::kCameraYawOffset}),
m_serialCam(9600) { 
    // Number of bytes in one chunk of vision data
    m_serialCam.SetReadBufferSize(65);
}

// This method will be called once per scheduler run
void Vision::Periodic() {
    if (VisionConstants::kShouldUseVision) {
        UpdateData();

        if (m_data.isDetected) {
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

void Vision::UpdateData() {
    std::cout << "Readable bytes: " << m_serialCam.GetBytesReceived() << "\n";
    frc::SmartDashboard::PutNumber("Readable bytes", m_serialCam.GetBytesReceived());
    if (m_serialCam.GetBytesReceived() >= 65) {
        char* buffer = 0;
        int bytesRead = m_serialCam.Read(buffer, 65);
        std::cout << "Bytes read: " << bytesRead << "\n";
        frc::SmartDashboard::PutNumber("Bytes read", bytesRead);
        m_data = *reinterpret_cast<VisionData*>(buffer);
    }

    frc::SmartDashboard::PutBoolean("Detected", m_data.isDetected);
    frc::SmartDashboard::PutNumber("Tag ID", m_data.ID);
    frc::SmartDashboard::PutNumber("Last timestamp", m_data.lastTimestamp);
    frc::SmartDashboard::PutNumber("Vision X distance", m_data.translationMatrix[0]);
    frc::SmartDashboard::PutNumber("Vision Y distance", m_data.translationMatrix[1]);
    frc::SmartDashboard::PutNumber("Vision Z distance", m_data.translationMatrix[2]);
    frc::SmartDashboard::PutNumber("Rotation X", m_data.rotationMatrix[0]);
    frc::SmartDashboard::PutNumber("Rotation Y", m_data.rotationMatrix[1]);
    frc::SmartDashboard::PutNumber("Rotation Z", m_data.rotationMatrix[2]);
}