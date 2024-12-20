#pragma once

#include <iostream>
#include <map>
#include <rev/SparkMax.h>

// Shooter Constants
namespace ShooterConstants {
    constexpr int kTopRollerMotorID {14};
    constexpr int kBottomRollerMotorID {13};
    constexpr int kLoaderMotorID {15};
    constexpr int kRotationMotorID {16};
    constexpr int kLeftLimitSwitchPort {2};
    constexpr int kRightLimitSwitchPort {3};

    constexpr double kPRotation {0.125};
    constexpr double kIRotation {0};
    constexpr double kDRotation {0.0};
    constexpr auto kGRotation {0.3_V};
    constexpr auto kSRotation {1.0_V};
    constexpr auto kVRotation {3.0_V * 1.0_s / 1.0_rad};
    constexpr auto kARotation {0.0_V * 1.0_s * 1.0_s / 1.0_rad};

    constexpr double kFeedforward {1.0};

    // In degrees
    constexpr auto kMaxRotationVelocity {180.0_deg / 1.0_s};
    constexpr auto kMaxRotationAcceleration {180.0_deg / 1.0_s / 1.0_s};

    constexpr double kShootPower {1.0};  // 0.8
    // constexpr double kShootPowerSlow {0.8};
    // constexpr double kShootPower {0.30};
    constexpr auto kRevUpTime {2.5_s};  // Max is 6400 RPM
    constexpr auto kShootTime {1.25_s};

    constexpr auto kAxleToCenterDistance {0.0741_m}; //meow - trillian
    constexpr auto kAxleToGroundDistance {0.2185_m};
    constexpr auto kSpeakerHeightTarget {1.9074_m};
    // Intake load
    constexpr auto kLoadTarget {0.0_deg};
    // constexpr auto kLoadTarget {25.0_deg};
    // constexpr auto kLoadTarget {40.0_deg};
    constexpr double kLoadPower {0.5};
    constexpr double kUnloadPower {-0.25};
    constexpr auto kUnloadTime {0.2_s};
    // Direct shooter load
    // constexpr auto kDirectLoadTarget {59.0_deg};
    constexpr auto kDirectLoadTarget {57.0_deg};
    constexpr double kDirectLoadPower {-0.25};
    
    // constexpr auto kStartingConfigTarget {59.0_deg};
    constexpr auto kStartingConfigTarget {57.0_deg};
    constexpr auto kRetractTarget {40.0_deg};
    // constexpr auto kCloseTarget {59.0_deg};
    // constexpr auto kAutoScoreTarget {59.0_deg};
    constexpr auto kCloseTarget {57.0_deg};
    constexpr auto kAutoScoreTarget {57.0_deg};
    constexpr auto kMidTarget {45.0_deg};
    constexpr auto kFarTarget {15.0_deg};
    constexpr double kRotationOffset {86.0 / 360.0};
    constexpr double kRotationConversion {360.0};
    constexpr bool kRotationInverted {false};
    constexpr unsigned int kRotationCurrentLimit {30};
    constexpr unsigned int kRollerCurrentLimit {50};
    constexpr unsigned int kLoaderCurrentLimit {30};
    constexpr auto kRotationStopDistance {5.0_deg};
    constexpr auto kRotationIdleTolerance {1.5_deg};

    constexpr bool kRollerInverted {true};

    constexpr auto kIdleMode {rev::spark::SparkBaseConfig::IdleMode::kBrake};

    const std::vector<units::meter_t> kShooterKnownDistances {
        2.0_m, 
        3.0_m, 
        4.0_m, 
        5.0_m,
        6.0_m
    };

    const std::vector<units::degree_t> kShooterKnownAngles {
        57.0_deg, 
        50.0_deg, 
        45.0_deg, 
        37.0_deg,
        32.0_deg
    };

}