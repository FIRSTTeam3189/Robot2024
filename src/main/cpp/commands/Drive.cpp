// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Drive.h"

Drive::Drive(frc2::CommandJoystick *joystick, SwerveDrive *swerveDrive, DriveState driveState, units::degree_t arbitraryAngle) :
m_bill(joystick),
m_swerveDrive(swerveDrive),
m_rotationPIDController(SwerveDriveConstants::kPRot, SwerveDriveConstants::kIRot, SwerveDriveConstants::kDRot),
m_driveState(driveState),
m_arbitraryAngle(arbitraryAngle),
m_allianceSide(frc::DriverStation::Alliance::kBlue) {
  // Use addRequirements() here to declare subsystem dependencies.
  (void)AutoConstants::kAutonomousPaths[0];
  (void)VisionConstants::kSyncBytes[0];
  AddRequirements(swerveDrive);

  if (frc::DriverStation::GetAlliance())
    m_allianceSide = frc::DriverStation::GetAlliance();

  // 1 degree position tolerance
  m_rotationPIDController.SetTolerance(1.0);
  m_rotationPIDController.EnableContinuousInput(-180, 180);

  m_rotationPKey = "Robot Rotation P";
  m_rotationIKey = "Robot Rotation I";
  m_rotationDKey = "Robot Rotation D";

  frc::Preferences::SetDouble(m_rotationPKey, SwerveDriveConstants::kPRot);
  frc::Preferences::SetDouble(m_rotationIKey, SwerveDriveConstants::kIRot);
  frc::Preferences::SetDouble(m_rotationDKey, SwerveDriveConstants::kDRot);
}

units::angular_velocity::radians_per_second_t Drive::GetDesiredRotationalVelocity() {
  // Get raw (-1.0 to 1.0) joystick positions for x and y axis
  // Left, up are -1.0; right, down are 1.0
  // Inverted so forward on joystick is down the field
  // If red alliance, flip 180
  m_allianceSide = frc::DriverStation::GetAlliance();
  double joystickX = 0.0, joystickY = 0.0;
  if (m_allianceSide) {
    if (m_allianceSide.value() == frc::DriverStation::Alliance::kRed) {
      frc::SmartDashboard::PutString("Alliance", "red");
      joystickX = m_bill->GetRawAxis(OperatorConstants::kAxisRightStickY);
      joystickY = m_bill->GetRawAxis(OperatorConstants::kAxisRightStickX);
    } else {
      frc::SmartDashboard::PutString("Alliance", "blue");
      joystickX = -(m_bill->GetRawAxis(OperatorConstants::kAxisRightStickY));
      joystickY = -(m_bill->GetRawAxis(OperatorConstants::kAxisRightStickX));
    }
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

  // converts a given or desired coordinate to a degree on the unit circle

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

  double joystickX = 0.0, joystickY = 0.0;
  m_allianceSide = frc::DriverStation::GetAlliance();
  if (m_allianceSide) {
    if (m_allianceSide.value() == frc::DriverStation::Alliance::kRed) {
      joystickX = -m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickY);
      joystickY = -m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickX);
    } else {
      joystickX = m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickY);
      joystickY = m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickX);
    }
  }

  // the controller values get swapped (-m_bill) to drive field relative based on alliance

  joystickX = pow(joystickX, 3.0);
  joystickY = pow(joystickY, 3.0);

  // joystick parameters

  units::meters_per_second_t xSpeed, ySpeed;

  xSpeed = (m_xSpeedLimiter.Calculate(joystickX) * SwerveDriveConstants::kMaxSpeed);
  ySpeed = (m_ySpeedLimiter.Calculate(joystickY) * SwerveDriveConstants::kMaxSpeed);

  // calculate spped based on the defined limits

  if (fabs(joystickX) < .05)
    xSpeed = 0_mps;
  if (fabs(joystickY) < .05)
    ySpeed = 0_mps;

  //ignore joystick noise (slight rotation)

  frc::SmartDashboard::PutNumber("X speed", xSpeed.value());
  frc::SmartDashboard::PutNumber("Y speed", ySpeed.value());
  units::radians_per_second_t rot;

  // If using atan2 control, where right joystick angle == robot heading angle
  // Also, if trying to align to speaker, angle is calculated by pose estimator

  switch (m_driveState) {
    case(DriveState::HeadingControl) :
      rot = GetDesiredRotationalVelocity();
      break;
    case(DriveState::SpeakerAlign) :
      rot = GetRotVelSpeakerAlign();
      break;
    case(DriveState::RotationVelocityControl) :
    {
      double joystickRotX = -m_bill->GetRawAxis(OperatorConstants::kAxisRightStickX);
      rot = (fabs(joystickRotX) < .05) ? 0.0_rad / 1.0_s : (m_xSpeedLimiter.Calculate(joystickRotX) * SwerveDriveConstants::kMaxAngularVelocity);
      break;
    }

    // if the robot is going to align to some angle like the speaker, amp, or source load
    case(DriveState::ArbitraryAngleAlign) :
      rot = -units::angular_velocity::radians_per_second_t{
              m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerveDrive->GetNormalizedYaw().value(), m_arbitraryAngle.value()))
              * SwerveDriveConstants::kMaxAngularVelocity};
      break;
    case(DriveState::SourceAlign) :
      {
        m_allianceSide = frc::DriverStation::GetAlliance();
        auto angle = 0.0_deg;
        if (m_allianceSide) {
          if (m_allianceSide.value() == frc::DriverStation::Alliance::kBlue) {
            angle = SwerveDriveConstants::kBlueSourceAlignTarget;
          } else {
            angle = SwerveDriveConstants::kRedSourceAlignTarget;
          }
        }
        rot = -units::angular_velocity::radians_per_second_t{
              m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerveDrive->GetNormalizedYaw().value(), angle.value()))
              * SwerveDriveConstants::kMaxAngularVelocity};
        break;
      }
    default:
      rot = units::angular_velocity::radians_per_second_t{0.0};
      break;
  }

  // based on drive state, set the rotation and velocity to do so

  m_swerveDrive->Drive(xSpeed, ySpeed, rot, true, frc::Translation2d{});
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
  m_allianceSide = frc::DriverStation::GetAlliance();
  frc::Pose3d tagPose = VisionConstants::kTagPoses.at(6);
  if (m_allianceSide) {
    if (m_allianceSide.value() == frc::DriverStation::Alliance::kRed) {
      tagPose = VisionConstants::kTagPoses.at(3);
    } else {
      tagPose = VisionConstants::kTagPoses.at(6);
    }
  }

  // which speaker it should align to based on alliance found by tag

  auto currentPose = m_swerveDrive->GetEstimatedPose();
  
  // Calculate the angle to rotate to for the robot to point towards the speaker
  // This is alliance-dependent 
  auto xDistance = tagPose.X() - currentPose.X();
  auto yDistance = tagPose.Y() - currentPose.Y();
  frc::SmartDashboard::PutNumber("Swerve align x distance", xDistance.value());
  frc::SmartDashboard::PutNumber("Swerve align y distance", yDistance.value());
  // x and y swapped when passed into atan function because our x is their y
  auto goalAngle = units::degree_t{units::radian_t{atan(yDistance.value() / xDistance.value())}};

  if (m_allianceSide) {
    if (m_allianceSide.value() == frc::DriverStation::Alliance::kRed) {
        goalAngle += 180.0_deg;
        if (goalAngle > 180.0_deg)
          goalAngle -= 360.0_deg;
    }
  }

  // Return next velocity in radians per second as calculated by PIDController and limited by rotLimiter
  units::angular_velocity::radians_per_second_t rot = 
              -units::angular_velocity::radians_per_second_t{
              m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerveDrive->GetNormalizedYaw().value(), goalAngle.value()))
              * SwerveDriveConstants::kMaxAngularVelocity};

  frc::SmartDashboard::PutNumber("Swerve auto align angle (teleop version)", goalAngle.value());
  return rot;
}
//for pid tunings
void Drive::UpdatePreferences() {
  if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
    m_rotationPIDController.SetP(frc::Preferences::GetDouble(m_rotationPKey, SwerveDriveConstants::kPRot));
    m_rotationPIDController.SetI(frc::Preferences::GetDouble(m_rotationIKey, SwerveDriveConstants::kIRot));
    m_rotationPIDController.SetD(frc::Preferences::GetDouble(m_rotationDKey, SwerveDriveConstants::kDRot));
  }
}