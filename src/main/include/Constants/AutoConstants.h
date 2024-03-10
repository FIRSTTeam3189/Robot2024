#pragma once

#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>

#define Pi 3.14159265358979323846

enum class StartingPosition { RedAmp, RedMid, RedSource, BlueAmp, BlueMid, BlueSource };

namespace AutoConstants {
    constexpr StartingPosition kStartingPosition {StartingPosition::RedMid};
    constexpr double kPTranslationAuto {4.5};
    constexpr double kITranslationAuto {0.0};
    constexpr double kDTranslationAuto {0.0};
    constexpr double kPRotationAuto {4.0};
    constexpr double kIRotationAuto {0.0};
    constexpr double kDRotationAuto {0.0};

    constexpr auto kMaxAutoSpeed{4.0_mps};
    constexpr auto kAlignAllowableDriveSpeed {0.25_mps};
    constexpr double kAutoAlignTolerance {2.5};
    constexpr auto kAutoRevUpTime {2.0_s};
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
        "Score 2 - Mid"sv,
        "Score 2 - Bottom"sv,
        "Score 3 - Top 1"sv,
        "Score 3 - Top 1-2"sv,
        "Score 3 - Mid 2"sv,
        "Score 3 - Mid 2-1"sv,
        "Score 3 - Mid 2-3"sv,
        "Score 3 - Mid 3 - Under"sv,
        "Score 3 - Bottom 5"sv,
        "Score 3 - Bottom 3-2"sv,
        "Score 4 - Mid 1-2-3"sv,
        "Sweep Auto"sv,
        "Test - Line"sv,
        "Test - Line Rotate"sv,
        "Test - S"sv,
        "Test - Left & Right"sv,
    };
}