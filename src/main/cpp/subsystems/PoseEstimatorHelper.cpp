// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PoseEstimatorHelper.h"

PoseEstimatorHelper::PoseEstimatorHelper() {
    // std::cout << "Helper constructing\n";
}

void PoseEstimatorHelper::SetPoseEstimator(frc::SwerveDrivePoseEstimator<4> *poseEstimator) {
    m_poseEstimator = poseEstimator;
    //set the member variable to the pose estimator variable
} 

void PoseEstimatorHelper::UpdatePoseEstimator(wpi::array<frc::SwerveModulePosition, 4U> modulePositions, frc::Rotation2d rotation) {
    m_poseEstimator->Update(rotation, modulePositions);
    UpdateRotation(rotation);
    //update the pose estimator with the rotation found by the encoders and the module positions
}

void PoseEstimatorHelper::SetActivePath(std::vector<frc::Pose2d> poses) {
    m_estimatedPose.GetObject("path")->SetPoses(poses);
}

void PoseEstimatorHelper::SetCurrentAutoPose(frc::Pose2d pose) {
    m_estimatedPose.SetRobotPose(pose);
}

void PoseEstimatorHelper::SetTargetAutoPose(frc::Pose2d pose) {
    m_estimatedPose.GetObject("target pose")->SetPose(pose);
}

frc::Pose2d PoseEstimatorHelper::GetEstimatedPose() {
    frc::Pose2d pigeonTrustingPose = frc::Pose2d{
        m_poseEstimator->GetEstimatedPosition().Translation(), m_rotation};
    return pigeonTrustingPose;
}

void PoseEstimatorHelper::UpdateRotation(frc::Rotation2d rotation) {
    m_rotation = rotation;
}

void PoseEstimatorHelper::ResetPose(frc::Rotation2d rotation, wpi::array<frc::SwerveModulePosition, 4> modulePositions, frc::Pose2d pose) {
    m_poseEstimator->ResetPosition(rotation, modulePositions, pose);
    //reset the positions based on newest values
}

void PoseEstimatorHelper::AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp, wpi::array<double, 3> visionMeasurementStdDevs) {
    m_poseEstimator->SetVisionMeasurementStdDevs(visionMeasurementStdDevs);

    // Currently setting pose rotation to estimated rotation rather than vision-deduced one
    // Gyro is already fairly accurate and prevents noisy rotation data for driving/auto
    // Sets pose angle to 0
    pose = pose.TransformBy(frc::Transform2d(0.0_m, 0.0_m, -pose.Rotation()));
    // Then sets pose angle to estimated pose angle
    pose = pose.TransformBy(frc::Transform2d(0.0_m, 0.0_m, m_rotation));
    
    m_visionPose.SetRobotPose(pose);
    frc::SmartDashboard::PutData("Vision pose", &m_visionPose);

    // Don't add vision measurement if distance to current one is >1m
    // frc::Pose2d currentPose = GetEstimatedPose();
    // units::meter_t distanceFromCurrentPose = units::meter_t{sqrt(pow(pose.X().value() - currentPose.X().value(), 2.0) 
    //                              + pow(pose.Y().value() - currentPose.Y().value(), 2.0))};
    // if (distanceFromCurrentPose < 1.0_m)
        // frc::SmartDashboard::PutNumber("Last vision timestamp", timestamp.value());
        m_poseEstimator->AddVisionMeasurement(pose, frc::Timer::GetFPGATimestamp());
}

void PoseEstimatorHelper::Periodic() {
    m_estimatedPose.SetRobotPose(GetEstimatedPose());
    frc::SmartDashboard::PutData("Estimated pose", &m_estimatedPose);
    frc::SmartDashboard::PutData("Auto field", &m_autoField);
}