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

#define PI 3.141592653

namespace SwerveDriveConstants {
    // PID, sensor IDs passed in via structs in namespace
    constexpr double kMPSToRPM {600.0};
    constexpr double kDEGToRAD {57.2957795131};
    constexpr double kWheelRadiusInches {2.0};
    constexpr double kWheelRadiusMeters {0.0508};
    constexpr double kWheelCircumferenceMeters {2.0 * PI * wheelRadiusMeters};
    constexpr double kEncoderSpeedGearRatio {8.1};
    constexpr double kEncoderTurnGearRatio {15.43};
    constexpr int kFalconEncoderTicksPerRevolution {2048};
    constexpr int kCancoderTicksPerRevolution {4096};

    // Coordinate plane distance in meters to each swerve drive
    // This has x-positive as forward, y-positive as left
    constexpr auto kXDistanceFromCenter {0.282575_m};
    constexpr auto kYDistanceFromCenter {0.282575_m};
    
    // 0-max ramp time, current limit in amps
    constexpr double loopRampRate {.1};
    constexpr double ampLimit {35.0};

    // SysID robot characterization values -- **varies by robot**
    constexpr auto ks {0.408_V};
    constexpr auto kv {3.206 * 1_V * 1_s / 1_m};
    constexpr auto ka {3.409 * 1_V * 1_s * 1_s / 1_m};

    // Robot maxes - approximated and varies by robot
    // Original max speed: 3.0
    constexpr auto kMaxSpeed {3.0_mps};
    constexpr auto kMaxAcceleration {2.0_mps_sq};
    constexpr units::radians_per_second_t kMaxAngularVelocity {pi};
    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration {pi / 2};

    constexpr double kPSpeed {0.0};
    constexpr double kISpeed {0.0};
    constexpr double kDSpeed {0.0};
    constexpr double kVSpeed {0.0};

    constexpr double kPAngle {2.0};
    constexpr double kIAngle {0.0};
    constexpr double kDAngle {0.0};
    constexpr double kVAngle {0.0};

    // These are for robot rotation, not wheel rotation
    constexpr double kPRot {0.05};
    constexpr double kIRot {0.0};
    constexpr double kDRot {0.005};

    // Sensor IDs for motors + encoders - labeled on robot
    constexpr int kGyroID {1};
    constexpr int kFrontRightAngleID {1};
    constexpr int kFrontRightSpeedID {2};
    constexpr int kBackRightAngleID {3};
    constexpr int kBackRightSpeedID {4};
    constexpr int kFrontLeftAngleID {5};
    constexpr int kFrontLeftSpeedID {6};
    constexpr int kBackLeftSpeedID {7};
    constexpr int kBackLeftAngleID {8};
    constexpr int kFrontLeftCancoderID {9};
    constexpr int kRightFrontCancoderID {10};
    constexpr int kBackLeftCancoderID {11};
    constexpr int kBackRightCancoderID {12};

    // Swerve angle offsets -- difference between actual degrees heading and absolute degree values
    constexpr double frontLeftOffset {235.5};
    constexpr double frontRightOffset {334.5};
    constexpr double backLeftOffset {9.8};
    constexpr double backRightOffset {83.5};

    // TalonFX Remote Sensor Type Settings
    constexpr int kRotorSensor{0};
    constexpr int kRemoteCANcoder{1};
    constexpr int kRemotePigeon2_Yaw{2};
    constexpr int kRemotePigeon2_Pitch{3};
    constexpr int kRemotePigeon2_Roll{4};
    constexpr int kFusedCANcoder{5};
    constexpr int kSyncCANcoder{6};
}