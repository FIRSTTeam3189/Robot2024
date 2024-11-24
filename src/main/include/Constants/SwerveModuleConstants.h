#pragma once

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/frequency.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <iostream>

#define Pi 3.14159265358979323846

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
    constexpr units::turn_t kFrontLeftOffset = units::turn_t{0.921143}; // 0.421143
    constexpr units::turn_t kFrontRightOffset = units::turn_t{-0.351562}; // 0.154785
    constexpr units::turn_t kBackLeftOffset = units::turn_t{0.115479}; // -0.384521
    constexpr units::turn_t kBackRightOffset = units::turn_t{0.332031}; // {-0.167969

    // Motor + sensor inversions
    constexpr bool kDriveMotorInverted = true;
    constexpr bool kAngleMotorInverted = true;
    constexpr bool kCANcoderInverted = false;

    // Encoder sensor range
    constexpr auto kCANcoderSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;

    // Motor neutral modes -- what they do when no power is applied
    constexpr auto kDriveNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    constexpr auto kAngleNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    constexpr double kDriveNeutralDeadband = 0.02;

    // TalonFX Remote Sensor Type Settings
    constexpr int kRotorSensor{0};
    constexpr int kRemoteCANcoder{1};
    constexpr int kRemotePigeon2_Yaw{2};
    constexpr int kRemotePigeon2_Pitch{3};
    constexpr int kRemotePigeon2_Roll{4};
    constexpr int kFusedCANcoder{5};
    constexpr int kSyncCANcoder{6};

    constexpr double kPDrive {3.0};
    constexpr double kIDrive {0.0};
    constexpr double kDDrive {0.0};
    constexpr double kVDrive {0.0};
    constexpr double kSDrive {0.0};

    constexpr double kPAngle {40.0};
    constexpr double kIAngle {0.0};
    constexpr double kDAngle {0.0};
    constexpr double kVAngle {0.0};
    constexpr double kSAngle {0.0};

    constexpr auto kMaxVoltage {10.0_V}; // volts
    // Current limits -- allows continuous operation at certain amps, or a peak of greater amps for <threshold time
    constexpr auto kAngleContinuousCurrentLimit = units::ampere_t{25.0};
    constexpr auto kAnglePeakCurrentLimit = units::ampere_t{40.0};
    constexpr auto kAnglePeakCurrentDuration = 0.1_s; // seconds
    constexpr bool kAngleEnableCurrentLimit = true;
    
    constexpr auto kDriveContinuousCurrentLimit = units::ampere_t{35.0};
    constexpr auto kDrivePeakCurrentLimit = units::ampere_t{60.0};
    constexpr auto kDrivePeakCurrentDuration = 0.1_s;
    constexpr bool kDriveEnableCurrentLimit = true;

    // PID, sensor IDs passed in via structs in namespace
    constexpr auto kMaxSpeed {4.0_mps};
    constexpr double kMPSToRPM {600.0};
    constexpr double kDEGToRAD {57.2957795131};
    constexpr double kWheelRadiusInches {2.0};
    constexpr double kWheelRadiusMeters {0.0508};
    constexpr double kWheelCircumferenceMeters {2.0 * Pi * kWheelRadiusMeters};
    constexpr double kDriveGearRatio {8.1};
    constexpr double kAngleGearRatio {15.43};
    constexpr double kRotationsPerMeter {1.0 / kWheelCircumferenceMeters};
    constexpr int kFalconEncoderTicksPerRevolution {2048};
    constexpr int kCANcoderTicksPerRevolution {4096};
    constexpr double kWheelCOF {1.200}; // estimated
}