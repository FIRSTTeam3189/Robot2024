// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision(PoseEstimatorHelper *helper) : m_helper(helper),
                                              m_data(),
                                              m_cameraToRobotTransform(VisionConstants::kCameraXOffset, VisionConstants::kCameraYOffset, VisionConstants::kCameraZOffset,
                                                                       frc::Rotation3d{0.0_rad, 0.0_rad, VisionConstants::kCameraYawOffset}),
                                              m_serialCam(VisionConstants::kBaudRate, frc::SerialPort::Port::kMXP)
{
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
void Vision::Periodic()
{
    if (VisionConstants::kShouldUseVision)
    {
        UpdateData();
    }
}

frc::Pose3d Vision::TagToCamera()
{
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
            0.0_deg});

    return tagPose.TransformBy(tagToCamera);
}

frc::Pose3d Vision::CameraToRobot(frc::Pose3d cameraPose)
{
    return cameraPose.TransformBy(m_cameraToRobotTransform);
}

void Vision::UpdatePosition()
{
    if (m_data.isDetected)
    {
        // Turn distances into robot pose
        frc::Pose3d cameraPose = TagToCamera();
        frc::Pose3d robotPose = CameraToRobot(cameraPose);
        units::meter_t tagDistance = units::meter_t{sqrt(pow(m_data.translationMatrix[0], 2.0) + pow(m_data.translationMatrix[1], 2.0))};
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

VisionData Vision::GetVisionData()
{
    return m_data;
}

// Processes one chunk of data from the m_buffer.
// Returns `std::nullopt` if we don't have enough bytes to read a VisionData struct.
// Will remove read bytes and sync sequence from the buffer after reading the VisionData struct.
std::optional<VisionData> Vision::ParseData()
{
    // Check if we have enough bytes to read the VisionData struct.
    if (m_buffer.size() < sizeof(VisionData) + VisionConstants::kSyncBytes.size())
    {
        return std::nullopt;
    }

    // Search for the sync bytes in the data.
    auto syncIter = std::search(m_buffer.begin(), m_buffer.end(), VisionConstants::kSyncBytes.begin(), VisionConstants::kSyncBytes.end());

    // Check if we found the sync bytes, if we did, then we'll check if we have enough bytes to read the VisionData struct.
    if (syncIter != m_buffer.end())
    {
        // If we have enough bytes to read the VisionData struct, then we'll read it and remove the bytes from the buffer.
        if (m_buffer.size() >= std::distance(m_buffer.begin(), syncIter) + sizeof(VisionData) + VisionConstants::kSyncBytes.size())
        {
            // Copy the VisionData struct from the buffer
            VisionData data;
            std::memcpy(&data, &m_buffer[std::distance(m_buffer.begin(), syncIter) + VisionConstants::kSyncBytes.size()], sizeof(VisionData));

            // Only print the data if we're in `Full Diagnostics`
            if (frc::Preferences::GetBoolean("Full Diagnostics", false))
            {
                // Print the buffer size
                std::cout << "Buffer size: " << m_buffer.size() << "\n";

                // Print the sync bytes in m_buffer and position of the VisionData struct in buffer
                std::cout << "Sync bytes position: " << std::distance(m_buffer.begin(), syncIter) << "\n";
                std::cout << "Sync bytes: ";
                for (auto i = 0; i < VisionConstants::kSyncBytes.size(); i++)
                {
                    std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)m_buffer[std::distance(m_buffer.begin(), syncIter) + i] << " ";
                }
                std::cout << "\n";
                std::cout << "VisionData position: " << std::distance(m_buffer.begin(), syncIter) + VisionConstants::kSyncBytes.size() << "\n";

                // Print the bytes used to construct the VisionData struct
                std::cout << "VisionData bytes: ";
                for (auto i = 0; i < sizeof(VisionData); i++)
                {
                    std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)m_buffer[std::distance(m_buffer.begin(), syncIter) + VisionConstants::kSyncBytes.size() + i] << " ";
                }
                std::cout << "\n";

                // Print out the members of the VisionData Struct
                std::cout << "VisionData: " << data.isDetected << " " << data.ID << " " << data.lastTimestamp << " " << data.translationMatrix[0] << " " << data.translationMatrix[1] << " " << data.translationMatrix[2] << " " << data.rotationMatrix[0] << " " << data.rotationMatrix[1] << " " << data.rotationMatrix[2] << "\n";
            }

            // Remove the bytes from the buffer
            m_buffer.erase(m_buffer.begin(), m_buffer.begin() + std::distance(m_buffer.begin(), syncIter) + sizeof(VisionData) + VisionConstants::kSyncBytes.size());
            return data;
        }
    }

    return std::nullopt;
}

void Vision::UpdateData()
{
    auto readableBytes = m_serialCam.GetBytesReceived();
    frc::SmartDashboard::PutNumber("Readable bytes", readableBytes);

    // Reserve extra bytes if we need to
    if (readableBytes + m_buffer.size() > m_buffer.capacity())
    {
        m_buffer.reserve(m_buffer.capacity() + readableBytes);
    }

    // Read the bytes from the serial port. account for the bytes already in the buffer.
    m_serialCam.Read(m_buffer.data() + m_buffer.size(), readableBytes);

    // Keep parsing data until we get a std::nullopt.
    while (auto data = ParseData())
    {
        m_data = *data;
    }

    // Check that the m_buffer size is less than kBufferSize.
    // If it is, then erase all the bytes from the buffer, and print a warning.
    if (m_buffer.size() > VisionConstants::kBufferSize)
    {
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