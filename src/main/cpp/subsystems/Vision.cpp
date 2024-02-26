// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision(PoseEstimatorHelper *helper) : 
m_helper(helper),
m_data(), 
m_cameraToRobotTransform(VisionConstants::kCameraXOffset, VisionConstants::kCameraYOffset, VisionConstants::kCameraZOffset,
  frc::Rotation3d{0.0_rad, 0.0_rad, VisionConstants::kCameraYawOffset}),
m_serialCam(VisionConstants::kBaudRate, frc::SerialPort::Port::kMXP) { 
    (void)AutoConstants::kAutonomousPaths[0];
    // Number of bytes in one chunk of vision data
    // m_serialCam.SetReadBufferSize(VisionConstants::kBufferSize);
    m_serialCam.SetReadBufferSize(VisionConstants::kBufferSize);
    frc::PowerDistribution PDH{};
    PDH.SetSwitchableChannel(true);

    // Reserve the size of the kBufferSize for the buffer
    m_buffer.reserve(VisionConstants::kBufferSize);
}

// This method will be called once per scheduler run
void Vision::Periodic() {
    if (VisionConstants::kShouldUseVision) {
        UpdateData();
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

void Vision::UpdatePosition(){
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

VisionData Vision::GetVisionData() {
    return m_data;
}

char* SubString(const char* input, int offset, int len, char* dest) {
  int input_len = strlen(input);

  if (offset + len > input_len) {
    return NULL;
  }

  strncpy(dest, input + offset, len);
  return dest;
}

void Vision::UpdateData() {
    // std::cout << "Readable bytes: " << m_serialCam.GetBytesReceived() << "\n";
    
    auto readableBytes = m_serialCam.GetBytesReceived();
    frc::SmartDashboard::PutNumber("Readable bytes", readableBytes);

    // Reserve extra bytes if we need to
    if (readableBytes + m_buffer.size() > m_buffer.capacity()) {
        m_buffer.reserve(m_buffer.capacity() + readableBytes);
    }

    // Read the bytes from the serial port. account for the bytes already in the buffer.
    m_serialCam.Read(m_buffer.data() + m_buffer.size(), readableBytes);

    // Check for sync bytes if we have enough bytes to read
    auto bytesNeeded = sizeof(VisionData) + VisionConstants::kSyncBytes.size();

    // If we have enough bytes to read, then we'll check for the sync bytes in the data.
    if (m_buffer.size() >= bytesNeeded) {
        // Search for the sync bytes in the data. 
        // If we find them, check we have enough bytes to read the VisionData struct.
        auto syncIter = std::search(m_buffer.begin(), m_buffer.end(), VisionConstants::kSyncBytes.begin(), VisionConstants::kSyncBytes.end());
        // Check if we found the sync bytes, if we did, then we'll check if we have enough bytes to read the VisionData struct.
        if (syncIter != m_buffer.end()) {
            // If we have enough bytes to read the VisionData struct, then we'll read it and remove the bytes from the buffer.
            if (m_buffer.size() >= std::distance(m_buffer.begin(), syncIter) + bytesNeeded) {
                // Copy the VisionData struct from the buffer
                std::memcpy(&m_data, &m_buffer[std::distance(m_buffer.begin(), syncIter) + VisionConstants::kSyncBytes.size()], sizeof(VisionData));
                // Print the bytes used to read the VisionData struct
                std::cout << "Bytes used: " << std::distance(m_buffer.begin(), syncIter) + bytesNeeded << "\n";
                // Remove the bytes from the buffer
                m_buffer.erase(m_buffer.begin(), m_buffer.begin() + std::distance(m_buffer.begin(), syncIter) + bytesNeeded);
            }
        }
    }

    // Check that the m_buffer size is less than kBufferSize.
    // If it is, then erase all the bytes from the buffer, and print a warning.
    if (m_buffer.size() > VisionConstants::kBufferSize) {
        m_buffer.clear();
        std::cout << "Vision buffer overflow, cleared buffer\n";
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