// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <vector>

#define Pi 3.14159265358979323846

namespace SwerveDriveConstants {
    constexpr int kGyroID {13};
    constexpr double kRadiansToDegreesMultiplier {180.0 / Pi};

    // Coordinate plane distance in meters to each swerve drive
    // This has x-positive as forward, y-positive as left
    constexpr auto kXDistanceFromCenter {0.282575_m};
    constexpr auto kYDistanceFromCenter {0.282575_m};

    constexpr double kGyroMountPoseYaw {25.1};

    static frc::SwerveDriveKinematics<4> kKinematics {
        frc::Translation2d{+SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{+SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{-SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{-SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter}
    };

    constexpr auto kMaxSpeed {1.0_mps};
    constexpr auto kMaxAcceleration {1.0_mps_sq};
    constexpr units::radians_per_second_t kMaxAngularVelocity {2.0 * Pi};
    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration {Pi};

    // SysID robot characterization values -- **varies by robot**
    constexpr auto ks {0.408_V};
    constexpr auto kv {3.206 * 1_V * 1_s / 1_m};
    constexpr auto ka {3.409 * 1_V * 1_s * 1_s / 1_m};

    // These are for robot rotation, not wheel rotation
    constexpr double kPRot {0.005};
    constexpr double kIRot {0.0};
    constexpr double kDRot {0.0};
}

namespace SwerveModuleConstants {
    // Sensor IDs for motors + encoders - labeled on robot
    constexpr int kFrontRightAngleID {1};
    constexpr int kFrontRightDriveID {2};
    constexpr int kBackRightAngleID {3};
    constexpr int kBackRightDriveID {4};
    constexpr int kFrontLeftAngleID {5};
    constexpr int kFrontLeftDriveID {6};
    constexpr int kBackLeftDriveID {7};
    constexpr int kBackLeftAngleID {8};
    constexpr int kFrontLeftCANcoderID {9};
    constexpr int kFrontRightCANcoderID {10};
    constexpr int kBackLeftCANcoderID {11};
    constexpr int kBackRightCANcoderID {12};

    // Swerve angle offsets -- difference between actual degrees heading and absolute degree values
    constexpr double kFrontLeftOffset {-0.171143};
    constexpr double kFrontRightOffset {-0.390137};
    constexpr double kBackLeftOffset {0.154785};
    constexpr double kBackRightOffset {0.425781};

    // Motor + sensor inversions
    constexpr bool kDriveMotorInverted = true;
    constexpr bool kAngleMotorInverted = true;
    constexpr bool kCANcoderInverted = false;

    // Encoder sensor range
    constexpr auto kCANcoderSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;

    // Motor neutral modes -- what they do when no power is applied
    constexpr auto kDriveNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    constexpr auto kAngleNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    // TalonFX Remote Sensor Type Settings
    constexpr int kRotorSensor{0};
    constexpr int kRemoteCANcoder{1};
    constexpr int kRemotePigeon2_Yaw{2};
    constexpr int kRemotePigeon2_Pitch{3};
    constexpr int kRemotePigeon2_Roll{4};
    constexpr int kFusedCANcoder{5};
    constexpr int kSyncCANcoder{6};

    // Robot maxes - approximated and varies by robot
    // Original max speed: 3.0
    constexpr auto kMaxDrive {3.0_mps};
    constexpr auto kMaxAcceleration {2.0_mps_sq};
    constexpr units::radians_per_second_t kMaxAngularVelocity {Pi};
    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration {Pi / 2};

    constexpr double kPDrive {1.0};
    constexpr double kIDrive {0.0};
    constexpr double kDDrive {0.0};
    constexpr double kVDrive {0.0};
    constexpr double kSDrive {0.0};

    constexpr double kPAngle {5.0};
    constexpr double kIAngle {0.0};
    constexpr double kDAngle {0.0};
    constexpr double kVAngle {0.0};
    constexpr double kSAngle {0.0};

    constexpr double kMaxVoltage {10.0};
    // Current limits -- allows continuous operation at certain amps, or a peak of greater amps for <threshold time
    constexpr int kAngleContinuousCurrentLimit = 25;
    constexpr int kAnglePeakCurrentLimit = 40;
    constexpr double kAnglePeakCurrentDuration = 0.1;
    constexpr bool kAngleEnableCurrentLimit = true;
    
    constexpr int kDriveContinuousCurrentLimit = 35;
    constexpr int kDrivePeakCurrentLimit = 60;
    constexpr double kDrivePeakCurrentDuration = 0.1;
    constexpr bool kDriveEnableCurrentLimit = true;

    // PID, sensor IDs passed in via structs in namespace
    constexpr double kMPSToRPM {600.0};
    constexpr double kDEGToRAD {57.2957795131};
    constexpr double kWheelRadiusInches {2.0};
    constexpr double kWheelRadiusMeters {0.0508};
    constexpr double kWheelCircumferenceMeters {2.0 * Pi * kWheelRadiusMeters};
    constexpr double kDriveGearRatio {8.1};
    constexpr double kAngleGearRatio {15.43};
    constexpr double kRotationsPerMeter {kDriveGearRatio / kWheelCircumferenceMeters};
    constexpr int kFalconEncoderTicksPerRevolution {2048};
    constexpr int kCANcoderTicksPerRevolution {4096};
}

namespace AutoConstants {
    constexpr double kPTranslationAuto {5.0};
    constexpr double kITranslationAuto {0.0};
    constexpr double kDTranslationAuto {0.0};
    constexpr double kPRotationAuto {5.0};
    constexpr double kIRotationAuto {0.0};
    constexpr double kDRotationAuto {0.0};
    constexpr auto kMaxAutoSpeed {4.5_mps};
    constexpr auto kDriveBaseRadius {0.3048_m};
    const pathplanner::HolonomicPathFollowerConfig autoConfig {
        pathplanner::PIDConstants(kPTranslationAuto, kITranslationAuto, kDTranslationAuto), // Translation PID constants
        pathplanner::PIDConstants(kPRotationAuto, kIRotationAuto, kDRotationAuto), // Rotation PID constants
        kMaxAutoSpeed, // Max module speed, in m/s
        kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
        pathplanner::ReplanningConfig() // Defaults to replanning if robot is not at starting point, doesn't replan if robot strays too far
    };
}
 // Shooter Constants
namespace ShooterConstants {
    constexpr int kRollerMotorID {13};
    constexpr int kLoaderMotorID {14};
    constexpr int kExtensionMotorID {15};
    constexpr int kRotationMotorID {16};
    constexpr double kPRotation {1.0};
    constexpr double kIRotation {0.0};
    constexpr double kDRotation {0.0};
    constexpr double kPExtension {1.0};
    constexpr double kIExtension {0.0};
    constexpr double kDExtension {0.0};

    constexpr double kLoadPower {0.5};
    constexpr double kLoadAngle {0.0};

    constexpr double kRetractTarget {0.0};
    constexpr double kCloseTarget {0};
    constexpr double kMidTarget {1000};
    constexpr double kFarTarget {2000};
    constexpr double kRotationOffset {0.0};
    constexpr double kRotationConversion {1.0};
    constexpr bool kRotationInverted {false};
    constexpr unsigned int kRotationCurrentLimit {30};
    constexpr double kRotationStopDistance {100.0};

    constexpr double kExtensionOffset {0.0};
    constexpr double kExtensionConversion {1.0};
    constexpr bool kExtensionInverted {false};
    constexpr double kExtensionStopDistance {100.0};

    constexpr int kUltrasonicPort {0};
    constexpr double kUltrasonicValueRange {196.8504}; // 5 meters in inches -- max range of sensor; used so sensor reports in inches
}

namespace LedConstants{
    constexpr int kCandleID {1};
}

namespace IntakeConstants{
    constexpr int kRotationMotorID {17};
    constexpr int kRollerMotorID {18};
    
    constexpr double kIntakePower {1.0};
    constexpr double kLoadPower {0.5};
    
    constexpr double kPRotation {0.5};
    constexpr double kIRotation {0};
    constexpr double kDRotation {0};

    constexpr double kRetractTarget {0};
    constexpr double kAmpTarget {1000};
    constexpr double kExtendTarget {2000};
    constexpr double kRotationOffset {0};
    constexpr double kRotationConversion {1.0};
    constexpr bool kRotationInverted {false};
    constexpr unsigned int kRotationCurrentLimit {30};
    constexpr double kRotationStopDistance {100.0};
    constexpr int kUltrasonicPort {0};
    // 5 meters in inches -- max range of sensor; used so sensor reports in inches
    constexpr double kUltrasonicValueRange {196.8504};
}

namespace OperatorConstants{
constexpr int kDriverControllerPort {0};
    constexpr int kCoDriverControllerPort {1};
    constexpr int kTestControllerPort {2};
    constexpr int kButtonIDSquare {1};
    constexpr int kButtonIDX {2};
    constexpr int kButtonIDCircle {3};
    constexpr int kButtonIDTriangle {4};
    constexpr int kButtonIDLeftBumper {5};
    constexpr int kButtonIDRightBumper {6};
    constexpr int kButtonIDLeftTrigger {7};
    constexpr int kButtonIDRightTrigger {8};
    constexpr int kButtonIDCreate {9};
    constexpr int kButtonIDMenu {10};
    constexpr int kButtonIDLeftStick {11};
    constexpr int kButtonIDRightStick {12};
    constexpr int kButtonIDPlaystation {13};
    constexpr int kButtonIDTouchpad {14};
    constexpr int kButtonIDMicrophone {15};

    constexpr int kAxisLeftStickX {0};
    constexpr int kAxisLeftStickY {1};
    constexpr int kAxisRightStickX {2};
    constexpr int kAxisLeftTrigger {3};
    constexpr int kAxisRightTrigger {4};
    constexpr int kAxisRightStickY {5};
}

namespace VisionConstants {
    constexpr wpi::array<double, 3> kEncoderTrustCoefficients {0.1, 0.1, 0.1};
    constexpr wpi::array<double, 3> kVisionTrustCoefficients {0.5, 0.5, 0.5};
    constexpr double kVisionStdDevPerMeter {0.1};
    constexpr units::meter_t kCameraXOffset {0.0_m};
    constexpr units::meter_t kCameraYOffset {0.0_m};
    constexpr units::meter_t kCameraZOffset {0.0_m};
    constexpr units::radian_t kCameraYawOffset {0.0_rad};
    constexpr bool kShouldUseVision {false};

    const std::vector<frc::Pose3d> kTagPoses {
        frc::Pose3d{0.0_m, 0.0_m, 0.0_m, frc::Rotation3d{0.0_rad, 0.0_rad, 0.0_rad}}
    };
}

namespace ClimberConstants{
    //FIND IDS - krishna 
    constexpr int kleftServoID {0}; 
    constexpr int krightServoID {0}; 
    constexpr int krightMotorID {0};
    constexpr int klefttMotorID {0};
    constexpr int kInvertMotor {false};
    constexpr double kExtendMotorSpeed {0.5};
    constexpr double kExtendServoAngle {0.0};
    constexpr double kRetractMotorSpeed {-0.5};
    constexpr double kRetractServoAngle {0.0};
}