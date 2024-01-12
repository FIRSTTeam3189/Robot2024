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
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/area.h>
#include <units/capacitance.h>
#include <units/charge.h>
#include <units/concentration.h>
#include <units/conductance.h>
#include <units/current.h>
#include <units/curvature.h>
#include <units/data.h>
#include <units/data_transfer_rate.h>
#include <units/density.h>
#include <units/dimensionless.h>
#include <units/energy.h>
#include <units/force.h>
#include <units/frequency.h>
#include <units/illuminance.h>
#include <units/impedance.h>
#include <units/inductance.h>
#include <units/length.h>
#include <units/luminous_flux.h>
#include <units/luminous_intensity.h>
#include <units/magnetic_field_strength.h>
#include <units/magnetic_flux.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/power.h>
#include <units/pressure.h>
#include <units/radiation.h>
#include <units/solid_angle.h>
#include <units/substance.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/volume.h>

#define PI 3.141592653

namespace SwerveDriveConstants {
    // Coordinate plane distance in meters to each swerve drive
    // This has x-positive as forward, y-positive as left
    constexpr auto kXDistanceFromCenter {0.282575_m};
    constexpr auto kYDistanceFromCenter {0.282575_m};

    // SysID robot characterization values -- **varies by robot**
    constexpr auto ks {0.408_V};
    constexpr auto kv {3.206 * 1_V * 1_s / 1_m};
    constexpr auto ka {3.409 * 1_V * 1_s * 1_s / 1_m};

    // These are for robot rotation, not wheel rotation
    constexpr double kPRot {0.05};
    constexpr double kIRot {0.0};
    constexpr double kDRot {0.005};
}

namespace SwerveModuleConstants {
    // Sensor IDs for motors + encoders - labeled on robot
    constexpr int kGyroID {1};
    constexpr int kFrontRightAngleID {1};
    constexpr int kFrontRightDriveID {2};
    constexpr int kBackRightAngleID {3};
    constexpr int kBackRightDriveID {4};
    constexpr int kFrontLeftAngleID {5};
    constexpr int kFrontLeftDriveID {6};
    constexpr int kBackLeftDriveID {7};
    constexpr int kBackLeftAngleID {8};
    constexpr int kFrontLeftCancoderID {9};
    constexpr int kRightFrontCancoderID {10};
    constexpr int kBackLeftCancoderID {11};
    constexpr int kBackRightCancoderID {12};

    // Swerve angle offsets -- difference between actual degrees heading and absolute degree values
    constexpr double kFrontLeftOffset {235.5};
    constexpr double kFrontRightOffset {334.5};
    constexpr double kBackLeftOffset {9.8};
    constexpr double kBackRightOffset {83.5};

    // Motor + sensor inversions
    constexpr bool kDriveMotorInverted = false;
    constexpr bool kAngleMotorInverted = false;
    constexpr bool kAbsoluteEncoderInverted = false;

    // Encoder sensor range
    constexpr auto kAbsoluteEncoderSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;

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
    constexpr units::radians_per_second_t kMaxAngularVelocity {PI};
    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration {PI / 2};

    constexpr double kPDrive {1.0};
    constexpr double kIDrive {0.0};
    constexpr double kDDrive {0.0};
    constexpr double kVDrive {0.0};
    constexpr double kSDrive {0.0};

    constexpr double kPAngle {2.0};
    constexpr double kIAngle {0.0};
    constexpr double kDAngle {0.0};
    constexpr double kVAngle {0.0};
    constexpr double kSAngle {0.0};

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
    constexpr double kWheelCircumferenceMeters {2.0 * PI * kWheelRadiusMeters};
    constexpr double kDriveGearRatio {8.1};
    constexpr double kAngleGearRatio {15.43};
    constexpr double kRotationsPerMeter {kDriveGearRatio / kWheelCircumferenceMeters};
    constexpr int kFalconEncoderTicksPerRevolution {2048};
    constexpr int kCancoderTicksPerRevolution {4096};
}