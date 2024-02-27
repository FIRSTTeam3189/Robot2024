#pragma once

#include <iostream>

namespace IntakeConstants{
    constexpr int kRotationMotorID {17};
    constexpr int kRollerMotorID {18};
    constexpr int kLimitSwitchPort {0};
    
    constexpr double kIntakePower {1.0};
    constexpr double kAmpScorePower {-1.0};
    constexpr double kLoadPower {0.5};

    constexpr unsigned int kRotationCurrentLimit {40};
    constexpr unsigned int kRollerCurrentLimit {30};

    // constexpr double kPRotation {0.005};
    // constexpr double kIRotation {0.00000};
    // constexpr double kDRotation {0.01};
    // constexpr auto kSRotation {0.84149_V};
    // constexpr auto kGRotation {0.52939_V};
    // constexpr auto kVRotation {0.015044_V * 1.0_s / 1.0_rad};
    // constexpr auto kARotation {0.0006516_V * 1.0_s * 1.0_s / 1.0_rad};

    constexpr double kPRotation {0.02};
    constexpr double kIRotation {0.0};
    constexpr double kDRotation {0.0};
    constexpr auto kSRotation {1.0_V};
    constexpr auto kGRotation {0.5_V};
    constexpr auto kVRotation {1.0_V * 1.0_s / 1.0_rad};
    constexpr auto kARotation {0.0_V * 1.0_s * 1.0_s / 1.0_rad};

    constexpr double kFeedforward {1.0};

    // In degrees
    constexpr auto kMaxRotationVelocity {180.0_deg / 1.0_s};
    constexpr auto kMaxRotationAcceleration {480.0_deg / 1.0_s / 1.0_s};

    constexpr bool kRollerInverted {true};
    constexpr auto kRetractTarget {80.0_deg};
    constexpr auto kAmpTarget {70.0_deg};
    constexpr auto kExtendTarget {-35.0_deg};
    constexpr double kRotationOffset {260.0};
    constexpr double kRotationConversion {360.0};
    constexpr bool kRotationInverted {false};
    constexpr bool kRotationMotorInverted {true};
    constexpr auto kRotationStopDistance {5.0_deg};
}