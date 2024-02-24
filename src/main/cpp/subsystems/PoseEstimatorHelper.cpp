// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PoseEstimatorHelper.h"

PoseEstimatorHelper::PoseEstimatorHelper() {
    (void)AutoConstants::kAutonomousPaths[0];
    std::cout << "Helper constructing\n";
}

void PoseEstimatorHelper::SetPoseEstimator(frc::SwerveDrivePoseEstimator<4> *poseEstimator) {
    m_poseEstimator = poseEstimator;
} 

void PoseEstimatorHelper::UpdatePoseEstimator(wpi::array<frc::SwerveModulePosition, 4U> modulePositions, frc::Rotation2d rotation) {
    m_poseEstimator->Update(rotation, modulePositions);
    m_field.SetRobotPose(GetEstimatedPose());
    frc::SmartDashboard::PutData("Field", &m_field);
}

frc::Pose2d PoseEstimatorHelper::GetEstimatedPose() {
    // auto pose = m_poseEstimator->GetEstimatedPosition();
    // frc::SmartDashboard::PutNumber("Estimated x", pose.X().value());
    // frc::SmartDashboard::PutNumber("Estimated y", pose.Y().value());
    // frc::SmartDashboard::PutNumber("Estimated rot", pose.Rotation().Degrees().value());
    return m_poseEstimator->GetEstimatedPosition();
}

void PoseEstimatorHelper::ResetPose(frc::Rotation2d rotation, wpi::array<frc::SwerveModulePosition, 4> modulePositions, frc::Pose2d pose) {
    m_poseEstimator->ResetPosition(rotation, modulePositions, pose);
}

void PoseEstimatorHelper::AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp, wpi::array<double, 3> visionMeasurementStdDevs) {
    m_poseEstimator->SetVisionMeasurementStdDevs(visionMeasurementStdDevs);

    frc::SmartDashboard::PutNumber("Vision robot x", pose.X().value());
    frc::SmartDashboard::PutNumber("Vision robot y", pose.Y().value());
    frc::SmartDashboard::PutNumber("Vision robot angle", pose.Rotation().Degrees().value());
    
    // Don't add vision measurement if distance to current one is >1m
    // frc::Pose2d currentPose = GetEstimatedPose();
    // units::meter_t distanceFromCurrentPose = units::meter_t{sqrt(pow(pose.X().value() - currentPose.X().value(), 2.0) 
    //                              + pow(pose.Y().value() - currentPose.Y().value(), 2.0))};
    // if (distanceFromCurrentPose < 1.0_m)
        frc::SmartDashboard::PutNumber("Last vision timestamp", timestamp.value());
        m_poseEstimator->AddVisionMeasurement(pose, timestamp);
}

void PoseEstimatorHelper::Periodic() {}