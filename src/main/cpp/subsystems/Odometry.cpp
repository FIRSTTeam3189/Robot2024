// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Odometry.h"

Odometry::Odometry(Vision *vision) : m_pigeon(SwerveDriveConstants::kGyroID, "rio"), m_vision(vision), m_poseEstimator() {
     m_pigeon.GetConfigurator().Apply(ctre::phoenix6::configs::Pigeon2Configuration{});

     m_pigeonConfigs.MountPose.MountPoseYaw = SwerveDriveConstants::kGyroMountPoseYaw;
// TODO
     m_pigeon.GetConfigurator().Apply(m_pigeonConfigs);
}

// This method will be called once per scheduler run
void Odometry::Periodic() {
    VisionData data = m_vision->GetVisionData();
    float distance = sqrt(pow(data.translationMatrix[0], 2.0f) + pow(data.translationMatrix[0], 2.0f));
    std::cout << "Distance to target in meters: " << distance << "\n";
    // Update trust before inputting vision measurements
    // Starts at std dev of 0.5m and increases by .1 for every meter away from the target
    double stdDev = 0.5 + ((double)distance * 0.1);
    std::cout << "Std dev trust rating: " << stdDev << "\n";
    // Rot trust is 100% because we're just using the gyro (not using vision rotation measurement)
    wpi::array<double, 3> stdDevArray{stdDev, stdDev, 0.0};
}
ctre::phoenix6::hardware::Pigeon2 *Odometry::GetPigeon(){
    return &m_pigeon;
}

ctre::phoenix6::configs::Pigeon2Configuration *Odometry::GetConfig(){
    return &m_pigeonConfigs;
}

void Odometry::SetPoseEstimator(frc::SwerveDrivePoseEstimator<4> *poseEstimator){
    m_poseEstimator = poseEstimator;
} 
        
 frc::SwerveDrivePoseEstimator<4> *Odometry::GetPoseEstimator(){
    return m_poseEstimator; 
}   

void Odometry::UpdateOdometry(wpi::array<frc::SwerveModulePosition, 4U> modulePositions){
    m_poseEstimator->Update(m_pigeon.GetRotation2d(), modulePositions);
}