#pragma once

#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>

#define Pi 3.14159265358979323846
 
namespace AutoConstants {
    constexpr double kPTranslationAuto {5.0};
    constexpr double kITranslationAuto {0.0};
    constexpr double kDTranslationAuto {0.0};
    constexpr double kPRotationAuto {5.0};
    constexpr double kIRotationAuto {0.0};
    constexpr double kDRotationAuto {0.0};
    constexpr auto kMaxAutoSpeed{4.0_mps};
    // Distance from robot center to furthest module
    constexpr auto kDriveBaseRadius {0.282575_m};
    const pathplanner::HolonomicPathFollowerConfig autoConfig {
        pathplanner::PIDConstants(kPTranslationAuto, kITranslationAuto, kDTranslationAuto), // Translation PID constants
        pathplanner::PIDConstants(kPRotationAuto, kIRotationAuto, kDRotationAuto), // Rotation PID constants
        kMaxAutoSpeed, // Max module speed, in m/s
        kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
        pathplanner::ReplanningConfig() // Defaults to replanning if robot is not at starting point, doesn't replan if robot strays too far
    };

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