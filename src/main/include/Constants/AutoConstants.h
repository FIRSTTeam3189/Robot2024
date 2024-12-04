#pragma once

#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/config/ModuleConfig.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/system/plant/DCMotor.h>

#define Pi 3.14159265358979323846

enum class StartingPosition { RedAmp, RedMid, RedSource, BlueAmp, BlueMid, BlueSource };

namespace AutoConstants {
    constexpr StartingPosition kStartingPosition {StartingPosition::BlueSource};

    constexpr auto kAlignAllowableDriveSpeed {0.25_mps};
    constexpr double kAutoAlignTolerance {2.5};
    constexpr auto kAutoRevUpTime {0.75_s}; // Max is 6400 RPM
    constexpr auto kAutoUnloadTime {0.25_s};
    constexpr auto kAutoUnloadPower {-0.25};
    constexpr auto kIntakeDuration {3.0_s};
    // Distance from robot center to furthest module
    constexpr auto kDriveBaseRadius {0.282575_m};
    constexpr auto kMaxAutoModuleSpeed{3.0_mps};
    // constexpr auto kMaxAutoModuleSpeed{2.0_mps};

    // Translation PID
    // constexpr double kPTranslationAuto {1.5};
    constexpr double kPTranslationAuto {4.5};
    constexpr double kITranslationAuto {0.0};
    constexpr double kDTranslationAuto {0.0};

    // Rotation PID
    // constexpr double kPRotationAuto {1.5};
    constexpr double kPRotationAuto {5.0};
    constexpr double kIRotationAuto {0.0};
    constexpr double kDRotationAuto {0.0};

    // nominalVoltage, stallTorque, stallCurrent, freeCurrent, freeSpeed, numMotors
    constexpr frc::DCMotor kDriveMotorConfig {
        12.0_V, units::newton_meter_t{5.84}, units::ampere_t{304.0}, units::ampere_t{1.5}, units::radians_per_second_t{636.6961104}
    };

    // Wheel radius, maxDriveVelocityMPS, wheelCOF, driveMotor, driveCurrentLimit, numMotors
    static pathplanner::ModuleConfig kAutoModuleConfig {
        units::meter_t{SwerveModuleConstants::kWheelRadiusMeters},
        SwerveModuleConstants::kMaxSpeed, 
        SwerveModuleConstants::kWheelCOF,
        kDriveMotorConfig,
        SwerveModuleConstants::kDrivePeakCurrentLimit,
        1
    };

    // units::kilogram_t mass, units::kilogram_square_meter_t MOI, ModuleConfig moduleConfig, units::meter_t trackwidth, units::meter_t wheelbase
    static pathplanner::RobotConfig kAutoRobotConfig {
        53.524_kg,
        units::kilogram_square_meter_t{5.500},
        kAutoModuleConfig,
        SwerveDriveConstants::kTrackwidth
        // SwerveDriveConstants::kWheelbase,
    };

    // // Auto config
    // const pathplanner::HolonomicPathFollowerConfig autoConfig {
    //     pathplanner::PIDConstants(kPTranslationAuto, kITranslationAuto, kDTranslationAuto), // Translation PID constants
    //     pathplanner::PIDConstants(kPRotationAuto, kIRotationAuto, kDRotationAuto), // Rotation PID constants
    //     kMaxAutoModuleSpeed, // Max module speed, in m/s
    //     kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
    //     pathplanner::ReplanningConfig() // Defaults to replanning if robot is not at starting point, doesn't replan if robot strays too far
    // };

    using namespace std::literals;
    constexpr std::array kAutonomousPaths {
        "Do Nothing - Mid"sv,
        "Score 1 - Top"sv,
        "Score 1 - Mid"sv,
        "Score 1 - Bottom"sv,
        "Score 1 - Bottom Backup"sv,
        "Score 2 - Top"sv,
        "Score 2 - Top Midfield"sv,
        "Score 2 - Mid"sv,
        "Score 2 - Bottom"sv,
        "Score 2 - Bottom Midfield"sv,
        "Score 3 - Top 1"sv,
        "Score 3 - Top 1-2"sv,
        "Score 3 - Mid 2"sv,
        "Score 3 - Mid 2-1"sv,
        "Score 3 - Mid 2-3"sv,
        "Score 3 - Mid 3 - Under"sv,
        "Score 3 - Bottom 5"sv,
        "Score 3 - Bottom 3-2"sv,
        "Score 4 - Mid 1-2-3"sv,
        "Score 4 - Mid 1-2-3 Speaker"sv,
        // "Score 4 - Mid 2-1-3"sv,
        "Score 4 - Mid No Rotate"sv,
        "Score 5 - Mid 1"sv,
        "Score 5 - Mid 5"sv,
        "Sweep Auto"sv,
        "Test - Curvy"sv,
        "Test - Line"sv,
        "Test - Line Rotate"sv,
        "Test - S"sv,
        "Test - Left & Right"sv,
    };
}