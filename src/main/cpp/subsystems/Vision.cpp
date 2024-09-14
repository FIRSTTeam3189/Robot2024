// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision(PoseEstimatorHelper *helper) : m_poseEstimator(helper),
                                              m_data(),
                                              m_cameraToRobotTransform(VisionConstants::kCameraXOffset, VisionConstants::kCameraYOffset, 0.0_m,
                                                                       frc::Rotation3d{0.0_deg, VisionConstants::kCameraPitchOffset, VisionConstants::kCameraYawOffset}),
                                              m_serialCam(VisionConstants::kBaudRate, frc::SerialPort::Port::kMXP),
                                              m_visionEnabled(VisionConstants::kShouldUseVision)
{
    
    // Number of bytes in one chunk of vision data
    // m_serialCam.SetReadBufferSize(VisionConstants::kBufferSize);
    m_serialCam.SetReadBufferSize(1);
    frc::PowerDistribution PDH{};
    PDH.SetSwitchableChannel(true);

    // Reserve the size of the kBufferSize for the buffer
    m_buffer.reserve(VisionConstants::kBufferSize);

    m_visionEnabledKey = "Vision Enabled";
    frc::Preferences::SetBoolean(m_visionEnabledKey, VisionConstants::kShouldUseVision);
}

// This method will be called once per scheduler run
void Vision::Periodic()
{   
    // m_visionEnabled = frc::Preferences::GetBoolean(m_visionEnabledKey);
    if (m_visionEnabled)
    {
        UpdateData();
    }
}

frc::Pose3d Vision::TagToCamera()
{
    frc::Pose3d tagPose = VisionConstants::kTagPoses.at(m_data.ID - 1);

    // Don't touch this you nice person
    // The frame of reference is right don't get confused
    // Invert the data for x on tags to the right since vision reports positive differences and
    // TransformBy adds so we need to subtract
    // Scuffed version of this code: https://www.chiefdelphi.com/t/how-to-properly-perform-3d-transformations-using-wpilibmath-libraries/425357/3
    // Rotate by inverse of z axis rotation reported by camera, then convert EDN axes to NWU
    frc::Transform3d tagToCamera = frc::Transform3d(
        units::meter_t{m_data.translationMatrix[2]},
        units::meter_t{-m_data.translationMatrix[0]},
        // units::meter_t{-m_data.translationMatrix[1]},
        0.0_m,
        frc::Rotation3d{
            0.0_deg,
            0.0_deg,
            // units::degree_t{-m_data.rotationMatrix[1]}});
            0.0_deg});
    
    auto cameraPose = tagPose.TransformBy(tagToCamera);
    frc::SmartDashboard::PutNumber("Camera pose x", cameraPose.X().value());
    frc::SmartDashboard::PutNumber("Camera pose y", cameraPose.Y().value());
    frc::SmartDashboard::PutNumber("Camera rotation", cameraPose.Rotation().ToRotation2d().Degrees().value());
    
    return cameraPose;
}

frc::Pose3d Vision::CameraToRobot(frc::Pose3d cameraPose)
{
    // 1. Scuffed, potentially wrong field relative way
    // auto x = cameraPose.X() + VisionConstants::kCameraXOffset;
    // auto y = cameraPose.Y() + VisionConstants::kCameraYOffset;
    // return frc::Pose2d{x, y, cameraPose.Rotation()};
    
    // 2. Probably more correct but maybe not working
    frc::Transform3d rotationCompensatedCameraToRobotTransform{m_cameraToRobotTransform.Translation(), 
        m_cameraToRobotTransform.Rotation().RotateBy(frc::Rotation3d(0.0_deg, 0.0_deg, units::degree_t{-m_data.rotationMatrix[1]}))};
    return cameraPose.TransformBy(rotationCompensatedCameraToRobotTransform);

    // frc::Transform3d pitchCompensatedCameraToRobotTransform{frc::Translation3d(), frc::Rotation3d{
    //     0.0_deg,
    //     VisionConstants::kCameraPitchOffset,
    //     0.0_deg,
    // }};
    // return cameraPose.TransformBy(pitchCompensatedCameraToRobotTransform);
    
    // 3. Don't do anything -- definitely not right but useful for test
    // return cameraPose;
}

void Vision::UpdatePosition()
{
    if (m_data.isDetected)
    {
        // Turn distances into robot pose
        frc::Pose3d cameraPose = TagToCamera();
        frc::Pose3d robotPose = CameraToRobot(cameraPose);
        auto tagDistance = units::meter_t{sqrt(pow(m_data.translationMatrix[0], 2.0) + pow(m_data.translationMatrix[1], 2.0))};
        // Calculate vision std devs based on tag distance
        double stdDevDistanceCompensation = tagDistance.value() * VisionConstants::kVisionStdDevPerMeter;
        auto baseVisionStdDevs = VisionConstants::kVisionTrustCoefficients;
        auto distanceCompensatedStdDevs = wpi::array<double, 3>{
            baseVisionStdDevs[0] + stdDevDistanceCompensation,
            baseVisionStdDevs[1] + stdDevDistanceCompensation,
            baseVisionStdDevs[2] + stdDevDistanceCompensation};

        auto timestamp = units::second_t{m_data.lastTimestamp};
        m_poseEstimator->AddVisionMeasurement(robotPose.ToPose2d(), timestamp, distanceCompensatedStdDevs);
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
                // std::cout << "Buffer size: " << m_buffer.size() << "\n";

                // // Print the sync bytes in m_buffer and position of the VisionData struct in buffer
                // std::cout << "Sync bytes position: " << std::distance(m_buffer.begin(), syncIter) << "\n";
                // std::cout << "Sync bytes: ";
                // for (auto i = 0; i < (int)VisionConstants::kSyncBytes.size(); i++)
                // {
                //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)m_buffer[std::distance(m_buffer.begin(), syncIter) + i] << " ";
                // }
                // std::cout << "\n";
                // std::cout << "VisionData position: " << std::distance(m_buffer.begin(), syncIter) + VisionConstants::kSyncBytes.size() << "\n";

                // // Print the bytes used to construct the VisionData struct
                // std::cout << "VisionData bytes: ";
                // for (auto i = 0; i < (int)sizeof(VisionData); i++)
                // {
                //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)m_buffer[std::distance(m_buffer.begin(), syncIter) + VisionConstants::kSyncBytes.size() + i] << " ";
                // }
                // std::cout << "\n";

                // // Print out the members of the VisionData Struct
                // std::cout << std::setprecision(15) << "VisionData: " << data.isDetected << " " << data.ID << " " << data.lastTimestamp << " " << data.translationMatrix[0] << " " << data.translationMatrix[1] << " " << data.translationMatrix[2] << " " << data.rotationMatrix[0] << " " << data.rotationMatrix[1] << " " << data.rotationMatrix[2] << "\n";
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

    auto currentLength = m_buffer.size();

    // Resize vector to the proper size to take in the bytes.
    m_buffer.resize(m_buffer.size() + readableBytes);

    // Read the bytes from the serial port. account for the bytes already in the buffer.
    auto bytesRead = m_serialCam.Read(m_buffer.data() + currentLength, readableBytes);
    m_buffer.resize(currentLength + bytesRead);

    // Keep parsing data until we get a std::nullopt.
    while (auto data = ParseData())
    {
        m_data = *data;
        UpdatePosition();
    }

    // Check that the m_buffer size is less than kBufferSize.
    // If it is, then erase all the bytes from the buffer, and print a warning.
    if (m_buffer.size() > VisionConstants::kBufferSize)
    {
        m_buffer.clear();
        std::cout << "Vision buffer overflow, cleared buffer\n";
    }

    frc::SmartDashboard::PutBoolean("Vision Detected", m_data.isDetected);
    frc::SmartDashboard::PutNumber("Vision Tag ID", m_data.ID);
    frc::SmartDashboard::PutNumber("Vision Last timestamp", m_data.lastTimestamp);
    frc::SmartDashboard::PutNumber("Vision X distance", m_data.translationMatrix[2]);
    frc::SmartDashboard::PutNumber("Vision Y distance", m_data.translationMatrix[0]);
    frc::SmartDashboard::PutNumber("Vision Z distance", m_data.translationMatrix[1]);
    frc::SmartDashboard::PutNumber("Vision Rotation X", m_data.rotationMatrix[0]);
    frc::SmartDashboard::PutNumber("Vision Rotation Y", m_data.rotationMatrix[1]);
    frc::SmartDashboard::PutNumber("Vision Rotation Z", m_data.rotationMatrix[2]);
}