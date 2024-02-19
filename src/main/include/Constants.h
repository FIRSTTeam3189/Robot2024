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
#include <iostream>
#include <vector>
#include <array>
#include <string>

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

    constexpr auto kMaxSpeed {2.0_mps};
    constexpr auto kMaxAcceleration {1.0_mps_sq};
    constexpr units::radians_per_second_t kMaxAngularVelocity {2.0 * Pi};
    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration {Pi};

    // SysID robot characterization values -- **varies by robot**
    constexpr auto ks {0.408_V};
    constexpr auto kv {3.206 * 1_V * 1_s / 1_m};
    constexpr auto ka {3.409 * 1_V * 1_s * 1_s / 1_m};

    // These are for robot rotation, not wheel rotation
    constexpr double kPRot {0.001};
    constexpr double kIRot {0.0};
    constexpr double kDRot {0.0};

    constexpr double kExtendDriveSpeed {0.3};
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
    constexpr double kFrontRightOffset {0.105469};
    constexpr double kBackLeftOffset {0.154785};
    constexpr double kBackRightOffset {0.425781};

    // Motor + sensor inversions
    constexpr bool kDriveMotorInverted = true;
    constexpr bool kAngleMotorInverted = true;
    constexpr bool kCANcoderInverted = false;

    // Encoder sensor range
    constexpr auto kCANcoderSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;

    // Motor neutral modes -- what they do when no power is applied
    constexpr auto kDriveNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    constexpr auto kAngleNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    // TalonFX Remote Sensor Type Settings
    constexpr int kRotorSensor{0};
    constexpr int kRemoteCANcoder{1};
    constexpr int kRemotePigeon2_Yaw{2};
    constexpr int kRemotePigeon2_Pitch{3};
    constexpr int kRemotePigeon2_Roll{4};
    constexpr int kFusedCANcoder{5};
    constexpr int kSyncCANcoder{6};

    constexpr double kPDrive {2.0};
    constexpr double kIDrive {0.0};
    constexpr double kDDrive {0.0};
    constexpr double kVDrive {0.0};
    constexpr double kSDrive {0.0};

    constexpr double kPAngle {10.0};
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
    constexpr auto kMaxAutoSpeed{4.5_mps};
    // Distance from robot center to furthest module
    constexpr auto kDriveBaseRadius {0.282575_m};
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
    constexpr double kPRotation {0.02};
    constexpr double kIRotation {0};
    constexpr double kDRotation {0.0};
    constexpr auto kSRotation {0.0_V};
    constexpr auto kGRotation {0.0_V};
    constexpr auto kVRotation {0.0_V * 0.0_s / 1.0_rad};
    constexpr auto kARotation {0.0_V * 0.0_s * 0.0_s / 1.0_rad};

    constexpr double kFeedforward {1.0};
    
    // In degrees
    constexpr auto kMaxRotationVelocity {120.0_deg / 1.0_s};
    constexpr auto kMaxRotationAcceleration {90.0_deg / 1.0_s / 1.0_s};

    constexpr double kPExtension {0.0};
    constexpr double kIExtension {0.0};
    constexpr double kDExtension {0.0};

    constexpr double kShootPower {1.0};

    constexpr auto kAxleToCenterDistance {0.0741_m};
    constexpr auto kAxleToGroundDistance {0.2185_m};
    constexpr auto kSpeakerHeightTarget {2.0574_m};
    // Intake load
    constexpr auto kLoadAngle {10.0_deg};
    constexpr double kLoadPower {0.5};
    constexpr double kUnloadPower {-0.25};
    // Direct shooter load
    constexpr auto kDirectLoadAngle {30.0_deg};
    constexpr double kDirectLoadPower {-0.5};
    constexpr auto kImmediateShootAngle {25.0_deg};

    constexpr double kRetractTarget {55.0};
    constexpr double kCloseTarget {55.0};
    constexpr double kMidTarget {30.0};
    constexpr double kFarTarget {15.0};
    constexpr double kRotationOffset {46.67};
    constexpr double kRotationConversion {360.0};
    constexpr bool kRotationInverted {false};
    constexpr unsigned int kRotationCurrentLimit {30};
    constexpr auto kRotationStopDistance {5.0_deg};

    constexpr bool kRollerInverted {true};

    constexpr double kExtensionOffset {0.0};
    constexpr double kExtensionConversion {1.0};
    constexpr bool kExtensionInverted {false};
    constexpr double kExtensionStopDistance {100.0};
    constexpr int kExtensionCountsPerRev {8192};
    constexpr auto kTrapExtensionAngle {0.0_deg};
    constexpr double kTrapExtension {0.0};

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
    constexpr double kAmpScorePower {-1.0};
    constexpr double kLoadPower {0.5};
    
    constexpr double kPRotation {0.005};
    constexpr double kIRotation {0.00000};
    constexpr double kDRotation {0.01};
    constexpr auto kSRotation {0.84149_V};
    constexpr auto kGRotation {0.52939_V};
    constexpr auto kVRotation {0.015044_V * 1.0_s / 1.0_rad};
    constexpr auto kARotation {0.0006516_V * 1.0_s * 1.0_s / 1.0_rad};

    constexpr double kFeedforward {1.0};

    // In degrees
    constexpr auto kMaxRotationVelocity {120.0_deg / 1.0_s};
    constexpr auto kMaxRotationAcceleration {90.0_deg / 1.0_s / 1.0_s};

    constexpr bool kRollerInverted {true};
    constexpr auto kRetractTarget {120.0_deg};
    constexpr auto kAmpTarget {105.0_deg};
    constexpr auto kExtendTarget {10.0_deg};
    constexpr double kRotationOffset {100.0};
    constexpr double kRotationConversion {360.0};
    constexpr bool kRotationInverted {false};
    constexpr bool kRotationMotorInverted {true};
    constexpr unsigned int kRotationCurrentLimit {30};
    constexpr auto kRotationStopDistance {5.0_deg};
    constexpr int kUltrasonicPort {1};
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
    constexpr bool kShouldUseVision {true};

    constexpr int kBaudRate {115200};
    constexpr int kBufferSize {1024};
    
    // Tag poses in order from 1 to 16
    const std::vector<frc::Pose3d> kTagPoses {
        frc::Pose3d{15.079_m, 0.246_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 120.0_deg}},
        frc::Pose3d{16.185_m, 0.884_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 120.0_deg}},
        frc::Pose3d{16.579_m, 4.983_m, 1.451_m, frc::Rotation3d{0.0_deg, 0.0_deg, 180.0_deg}},
        frc::Pose3d{16.579_m, 5.548_m, 1.451_m, frc::Rotation3d{0.0_deg, 0.0_deg, 180.0_deg}},
        frc::Pose3d{14.701_m, 8.204_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 270.0_deg}},
        frc::Pose3d{1.842_m, 8.204_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 270.0_deg}},
        frc::Pose3d{-0.038_m, 5.548_m, 1.451_m, frc::Rotation3d{0.0_deg, 0.0_deg, 0.0_deg}},
        frc::Pose3d{-0.038_m, 4.983_m, 1.451_m, frc::Rotation3d{0.0_deg, 0.0_deg, 0.0_deg}},
        frc::Pose3d{0.356_m, 0.884_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 60.0_deg}},
        frc::Pose3d{1.462_m, 0.246_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 60.0_deg}},
        frc::Pose3d{11.905_m, 3.713_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 300.0_deg}},
        frc::Pose3d{11.905_m, 4.498_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 60.0_deg}},
        frc::Pose3d{11.220_m, 4.105_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 60.0_deg}},
        frc::Pose3d{5.321_m, 4.105_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 0.0_deg}},
        frc::Pose3d{4.641_m, 4.498_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 120.0_deg}},
        frc::Pose3d{4.641_m, 3.713_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 240.0_deg}}
    };
}

namespace ClimberConstants{
    //FIND IDS - krishna 
    constexpr int kLeftServoID {0}; 
    constexpr int kRightServoID {1}; 
    constexpr int kRightMotorID {19};
    constexpr int kLefttMotorID {20};
    constexpr int kInvertMotor {false};
    constexpr double kExtendMotorSpeed {0.5};
    constexpr double kExtendServoAngle {0.0};
    constexpr double kRetractMotorSpeed {-0.5};
    constexpr double kRetractServoAngle {0.0};
}

namespace AutoConstants {
    using namespace std::literals;
    constexpr std::array kAutonomousPaths {
        "Do Nothing - Mid"sv,
        "Score 1 - Top"sv,
        "Score 1 - Mid"sv,
        "Score 1 - Bottom"sv,
        "Score 2 - Top"sv,
        "Score 2 - Skip First Top"sv,
        "Score 2 - Mid"sv,
        "Score 2 - Mid - Amp"sv,
        "Score 2 - Bottom"sv,
        "Score 3 - Top 1"sv,
        "Score 3 - Top - Amp"sv,
        "Score 3 - Top 1 - Amp x2"sv,
        "Score 3 - Skip First Top"sv,
        "Score 3 - Mid 2"sv,
        "Score 3 - Mid 3 - Under"sv,
        "Score 3 - Bottom 5"sv,
        "Sweep Auto"sv,
        "Test - Line"sv,
        "Test - Line Rotate"sv,
        "Test - S"sv
    };
}