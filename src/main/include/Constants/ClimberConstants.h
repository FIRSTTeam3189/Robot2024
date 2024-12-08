#pragma once

#include <iostream>
#include <units/current.h>

namespace ClimberConstants{
    constexpr int kRightMotorID {19};
    constexpr int kLeftMotorID {20};
    constexpr int kLimitSwitchPort {4};
    constexpr bool kInvertLeftMotor {false};
    constexpr bool kInvertRightMotor {false};
    constexpr double kExtendPower {0.5};
    constexpr double kBothExtendPower {0.4};
    constexpr double kRetractPower {-0.95};
    constexpr double kBothRetractPower {-0.95};
    constexpr int kCurrentLimit {100}; // Amps
    constexpr auto kIdleMode {rev::spark::SparkMaxConfig::IdleMode::kBrake};
    constexpr auto kSoftLimitForwardEnabled {true};
    constexpr auto kSoftLimitReverseEnabled {true};
    const bool kLeftMotorSoftLimitEnabled = true;
    const bool kRightMotorSoftLimitEnabled = true;
    constexpr double kSoftLimitForwardValue {50.0};
    constexpr double kLeftMotorSoftLimitReverseValue {2.0};
    constexpr double kRightMotorSoftLimitReverseValue {2.0};
}