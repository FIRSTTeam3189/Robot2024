// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/JoystickDrive.h"

JoystickDrive::JoystickDrive(frc2::CommandJoystick *joystick, SwerveDrive *swerveDrive, bool isSpecialHeadingMode, bool isFieldRelative) :
m_bill(joystick),
m_swerveDrive(swerveDrive),
m_rotationPIDController(SwerveDriveConstants::kPRot, SwerveDriveConstants::kIRot, SwerveDriveConstants::kDRot),
m_isSpecialHeadingMode(isSpecialHeadingMode),
m_isFieldRelative(isFieldRelative) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerveDrive);
  // 1 degree position tolerance
  m_rotationPIDController.SetTolerance(1.0);
  m_rotationPIDController.EnableContinuousInput(0, 360);
}

units::angular_velocity::radians_per_second_t JoystickDrive::GetDesiredRotationalVelocity() {
  // Get raw (-1.0 to 1.0) joystick positions for x and y axis
  // Left, up are -1.0; right, down are 1.0
  // Inverted so forward on joystick is down the field
  double joystickX = -m_bill->GetRawAxis(OperatorConstants::kAxisRightStickY);
  double joystickY = -m_bill->GetRawAxis(OperatorConstants::kAxisRightStickX);

  // Manual deadband to inputs greater than 2% only
  // TODO: Figure out if there's a better way to deadband
  if ((fabs(joystickX) < .05) && (fabs(joystickY) < .05)) 
    return units::angular_velocity::radians_per_second_t{0.0};

  // Convert joystick positions to goal angle in degrees
  // Normalized from -180, 180
  // Uses arctan2 function -- converts Cartesian coordinates (1, 1) to a polar angle (pi / 4), then multiplies by radians to degrees conversion
  // Converts rad to degrees
  double goalAngle = SwerveDriveConstants::kRadiansToDegreesMultiplier * atan2(joystickY, joystickX);

  frc::SmartDashboard::PutNumber("Robot Desired Angle", goalAngle);

  // Return next velocity in radians per second as calculated by PIDController and limited by rotLimiter
  units::angular_velocity::radians_per_second_t rot = 
              units::angular_velocity::radians_per_second_t{
              m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerveDrive->GetNormalizedYaw(), goalAngle))
              * SwerveDriveConstants::kMaxAngularVelocity};

  frc::SmartDashboard::PutNumber("Rotation PID Output", rot.value());
    
  // TODO: Check if necessary, should be redundant if SetTolerance works correctly
  // if (abs(m_swerveDrive->GetNormalizedYaw() - goalAngle) < 2.5) {
  //   rot = units::angular_velocity::radians_per_second_t{0.0};
  // }

  return rot;
}

// Called when the command is initially scheduled.
void JoystickDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void JoystickDrive::Execute() {
  double joystickX = -m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickY);
  double joystickY = -m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickX);

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
  if (m_isSpecialHeadingMode) {
    rot = GetDesiredRotationalVelocity();
    m_swerveDrive->Drive(xSpeed, ySpeed, rot, m_isFieldRelative, frc::Translation2d{});
  } else {
    double joystickRotX = -m_bill->GetRawAxis(OperatorConstants::kAxisRightStickX);
    rot = (fabs(joystickRotX) < .05) ? 0.0_rad / 1.0_s : (m_xSpeedLimiter.Calculate(joystickRotX) * 
                                                          SwerveDriveConstants::kMaxAngularVelocity);
    m_swerveDrive->Drive(xSpeed, ySpeed, rot, m_isFieldRelative, frc::Translation2d{});
  }
}

// Called once the command ends or is interrupted.
void JoystickDrive::End(bool interrupted) {
  m_swerveDrive->Stop();
}

// Returns true when the command should end.
bool JoystickDrive::IsFinished() {
  return false;
}
