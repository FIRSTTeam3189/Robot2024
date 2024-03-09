#pragma once

#include <iostream>

// Shooter Constants
namespace ShooterConstants {
    constexpr int kRollerMotorID {13};
    constexpr int kLoaderMotorID {14};
    constexpr int kRotationMotorID {16};
    constexpr int kLimitSwitchPort {1};

    constexpr double kPRotation {0.1};
    constexpr double kIRotation {0};
    constexpr double kDRotation {0.0};
    constexpr auto kGRotation {0.3_V};
    constexpr auto kSRotation {1.0_V};
    constexpr auto kVRotation {2.5_V * 1.0_s / 1.0_rad};
    constexpr auto kARotation {0.0_V * 1.0_s * 1.0_s / 1.0_rad};

    constexpr double kFeedforward {1.0};
    
    // In degrees
    constexpr auto kMaxRotationVelocity {180.0_deg / 1.0_s};
    constexpr auto kMaxRotationAcceleration {180.0_deg / 1.0_s / 1.0_s};

    constexpr double kShootPower {1.0};
    constexpr auto kRevUpTime {1.5_s};
    constexpr auto kShootTime {1.25_s};

    constexpr auto kAxleToCenterDistance {0.0741_m};
    constexpr auto kAxleToGroundDistance {0.2185_m};
    constexpr auto kSpeakerHeightTarget {2.0574_m};
    // Intake load
    constexpr auto kLoadTarget {30.0_deg};
    constexpr double kLoadPower {0.5};
    constexpr double kUnloadPower {-0.25};
    // Direct shooter load
    constexpr auto kDirectLoadTarget {50.0_deg};
    constexpr double kDirectLoadPower {-0.25};

    // Trap scoring
    constexpr double kLinearActuatorExtendPower {0.5};
    constexpr double kLinearActuatorRetractPower {-0.5};
    constexpr double kTrapLoadLoaderPower {0.2};
    constexpr double kTrapLoadRollerPower {0.2};
    constexpr double kTrapScoreLoaderPower {1.0};
    constexpr double kTrapScoreRollerPower {1.0};
    
    constexpr auto kStartingConfigTarget {65.0_deg};
    constexpr auto kRetractTarget {60.0_deg};
    constexpr auto kCloseTarget {60.0_deg};
    constexpr auto kAutoScoreTarget {60.0_deg};
    constexpr auto kMidTarget {30.0_deg};
    constexpr auto kFarTarget {15.0_deg};
    constexpr double kRotationOffset {50.0};
    constexpr double kRotationConversion {360.0};
    constexpr bool kRotationInverted {false};
    constexpr unsigned int kRotationCurrentLimit {40};
    constexpr unsigned int kRollerCurrentLimit {50};
    constexpr unsigned int kLoaderCurrentLimit {30};
    constexpr auto kRotationStopDistance {5.0_deg};
    constexpr auto kRotationIdleTolerance {1.5_deg};

    constexpr bool kRollerInverted {true};

    constexpr auto kIdleMode {rev::CANSparkMax::IdleMode::kBrake};
}