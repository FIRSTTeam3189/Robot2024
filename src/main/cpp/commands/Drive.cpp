// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Drive.h"

Drive::Drive(frc2::CommandJoystick *joystick, SwerveDrive *swerveDrive, 
                             bool isSpecialHeadingMode, bool isFieldRelative, bool shouldAlignSpeaker) :
m_bill(joystick),
m_swerveDrive(swerveDrive),
m_rotationPIDController(SwerveDriveConstants::kPRot, SwerveDriveConstants::kIRot, SwerveDriveConstants::kDRot),
m_isSpecialHeadingMode(isSpecialHeadingMode),
m_isFieldRelative(isFieldRelative),
m_shouldAlignSpeaker(shouldAlignSpeaker),
m_allianceSide(frc::DriverStation::GetAlliance()) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerveDrive);
  // 1 degree position tolerance
  m_rotationPIDController.SetTolerance(1.0);
  m_rotationPIDController.EnableContinuousInput(-180, 180);

  m_rotationPKey = "Robot Rotation P";
  m_rotationIKey = "Robot Rotation I";
  m_rotationDKey = "Robot Rotation D";

  frc::Preferences::InitDouble(m_rotationPKey, SwerveDriveConstants::kPRot);
  frc::Preferences::InitDouble(m_rotationIKey, SwerveDriveConstants::kIRot);
  frc::Preferences::InitDouble(m_rotationDKey, SwerveDriveConstants::kDRot);
}

units::angular_velocity::radians_per_second_t Drive::GetDesiredRotationalVelocity() {
  // Get raw (-1.0 to 1.0) joystick positions for x and y axis
  // Left, up are -1.0; right, down are 1.0
  // Inverted so forward on joystick is down the field
  // If red alliance, flip 180
  double joystickX, joystickY;
  if (m_allianceSide == frc::DriverStation::Alliance::kRed) {
    joystickX = m_bill->GetRawAxis(OperatorConstants::kAxisRightStickY);
    joystickY = m_bill->GetRawAxis(OperatorConstants::kAxisRightStickX);
  } else {
    joystickX = -m_bill->GetRawAxis(OperatorConstants::kAxisRightStickY);
    joystickY = -m_bill->GetRawAxis(OperatorConstants::kAxisRightStickX);
  }

  // Manual deadband to inputs greater than 5% only
  // If deadband detected (i.e. user is not giving rotation input), then set goalAngle as last desired angle
  if ((fabs(joystickX) < .05) && (fabs(joystickY) < .05)) {
    // m_goalAngle = m_lastAngle;
    return units::angular_velocity::radians_per_second_t{0.0};
  } else {
    // Convert joystick positions to goal angle in degrees
    // Normalized from -180, 180
    // Uses arctan2 function -- converts Cartesian coordinates (1, 1) to a polar angle (pi / 4), then multiplies by radians to degrees conversion
    // Converts rad to degrees
    m_goalAngle = SwerveDriveConstants::kRadiansToDegreesMultiplier * atan2(joystickY, joystickX);
  }
  m_lastAngle = m_goalAngle;

  frc::SmartDashboard::PutNumber("Robot Desired Angle", m_goalAngle);

  // Return next velocity in radians per second as calculated by PIDController and limited by rotLimiter
  units::angular_velocity::radians_per_second_t rot = 
              -units::angular_velocity::radians_per_second_t{
              m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerveDrive->GetNormalizedYaw().value(), m_goalAngle))
              * SwerveDriveConstants::kMaxAngularVelocity};

  frc::SmartDashboard::PutNumber("Rotation PID Output", rot.value());
    
  if (abs(m_swerveDrive->GetNormalizedYaw().value() - m_goalAngle) < 2.5) {
    rot = units::angular_velocity::radians_per_second_t{0.0};
  }

  return rot;
}

// Called when the command is initially scheduled.
void Drive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute() {
  UpdatePreferences();

  double joystickX, joystickY;
  if (m_allianceSide == frc::DriverStation::Alliance::kRed) {
    joystickX = -m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickY);
    joystickY = -m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickX);
  } else {
    joystickX = m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickY);
    joystickY = m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickX);
  }

  units::meters_per_second_t xSpeed, ySpeed;

  xSpeed = (m_xSpeedLimiter.Calculate(joystickX) * SwerveDriveConstants::kMaxSpeed);
  ySpeed = (m_ySpeedLimiter.Calculate(joystickY) * SwerveDriveConstants::kMaxSpeed);

  if (fabs(joystickX) < .05)
    xSpeed = 0_mps;
  if (fabs(joystickY) < .05)
    ySpeed = 0_mps;

  frc::SmartDashboard::PutNumber("X speed", xSpeed.value());
  frc::SmartDashboard::PutNumber("Y speed", ySpeed.value());
  // units::meters_per_second_t xSpeed = (fabs(joystickX) < .05) ? 0.0_mps : (m_xSpeedLimiter.Calculate(joystickX) * SwerveDriveConstants::kMaxSpeed);
  // units::meters_per_second_t ySpeed = (fabs(joystickY) < .05) ? 0.0_mps : (m_ySpeedLimiter.Calculate(joystickY) * SwerveDriveConstants::kMaxSpeed);
  units::radians_per_second_t rot;

  // If using atan2 control, where right joystick angle == robot heading angle
  // Also, if trying to align to speaker, angle is calculated by pose estimator
  if (m_isSpecialHeadingMode) {
    rot = GetDesiredRotationalVelocity();
  } else if (m_shouldAlignSpeaker) {
    rot = GetRotVelSpeakerAlign();
  } else {
    double joystickRotX = -m_bill->GetRawAxis(OperatorConstants::kAxisRightStickX);
    rot = (fabs(joystickRotX) < .05) ? 0.0_rad / 1.0_s : (m_xSpeedLimiter.Calculate(joystickRotX) * 
                                                          SwerveDriveConstants::kMaxAngularVelocity);
  }

  m_swerveDrive->Drive(xSpeed, ySpeed, rot, m_isFieldRelative, frc::Translation2d{});
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {
  m_swerveDrive->Stop();
}

// Returns true when the command should end.
bool Drive::IsFinished() {
  return false;
}

units::angular_velocity::radians_per_second_t Drive::GetRotVelSpeakerAlign() {
  frc::Pose3d tagPose;
    if (m_allianceSide == frc::DriverStation::Alliance::kRed) {
      tagPose = VisionConstants::kTagPoses.at(3);
    } else {
      tagPose = VisionConstants::kTagPoses.at(6);
    }

  auto currentPose = m_swerveDrive->GetEstimatedPose();
  
  // Calculate the angle to rotate to for the robot to point towards the speaker
  // This is alliance-dependent 
  auto xDistance = tagPose.X() - currentPose.X();
  auto yDistance = tagPose.Y() - currentPose.Y();
  frc::SmartDashboard::PutNumber("Swerve align x distance", xDistance.value());
  frc::SmartDashboard::PutNumber("Swerve align y distance", yDistance.value());
  // x and y swapped when passed into atan function because our x is their y
  auto goalAngle = units::degree_t{units::radian_t{atan(xDistance.value() / yDistance.value())}};

  if (m_allianceSide == frc::DriverStation::Alliance::kRed) {
      goalAngle += 180.0_deg;
      if (goalAngle > 180.0_deg)
        goalAngle -= 360.0_deg;
  }

  // Return next velocity in radians per second as calculated by PIDController and limited by rotLimiter
  units::angular_velocity::radians_per_second_t rot = 
              -units::angular_velocity::radians_per_second_t{
              m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerveDrive->GetNormalizedYaw().value(), goalAngle.value()))
              * SwerveDriveConstants::kMaxAngularVelocity};

  frc::SmartDashboard::PutNumber("Swerve auto align angle (teleop version)", goalAngle.value());
  return rot;
}

void Drive::UpdatePreferences() {
  if (frc::Preferences::GetBoolean("Tuning Mode?", false)) {
    m_rotationPIDController.SetP(frc::Preferences::GetDouble(m_rotationPKey, SwerveDriveConstants::kPRot));
    m_rotationPIDController.SetI(frc::Preferences::GetDouble(m_rotationIKey, SwerveDriveConstants::kIRot));
    m_rotationPIDController.SetD(frc::Preferences::GetDouble(m_rotationDKey, SwerveDriveConstants::kDRot));
  }
}