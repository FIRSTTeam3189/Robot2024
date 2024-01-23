// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision() : m_data() { 
  auto nti = nt::NetworkTableInstance::GetDefault();
  m_isDetectedTopic = nti.GetIntegerTopic("Vision/Detection");
  m_tagIDTopic = nti.GetIntegerTopic("Vision/AprilTag/ID");
  m_translationMatrixTopic = nti.GetFloatArrayTopic("Vision/AprilTag/TMatrix");
}

// This method will be called once per scheduler run
void Vision::Periodic() {
    if(VisionConstants::shouldUseVision) {
    int detection = m_isDetectedTopic.Subscribe(-1);
    }else{ 
        m_data.ID = m_IDTopic.Subscribe(-1).Get();
        float a[]{0.0f, 0.0f, 0.0f};
        span s{a,size(a)};

        //Finds the x and y distances, and finds the offset. Fixes the camera as if it was the center of the robot.
        m_data.translationMatrix = m_TranslationMatrixTopic.Subscribe(s).Get();
        m_data.translationMatrix[0] += VisionConstants::cameraXOffset;
        m_data.translationMatrix[1] += VisionConstants::cameraYOffset;
        frc::SmartDashboard::PutNumber("Vision X distance", m_data.translationMatrix[0]);
        frc::SmartDashboard::PutNumber("Vision Y distance", m_data.translationMatrix[1]);
    }
}

void Vision::GetVisionData(){
    return m_data;
}