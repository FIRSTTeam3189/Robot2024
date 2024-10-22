#pragma once

#include<iostream>

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
    constexpr double kCurrentLimit {100.0}; // Amps
    constexpr auto kIdleMode {rev::CANSparkMax::IdleMode::kBrake};
    constexpr auto kLeftMotorSoftLimitDirection {rev::CANSparkBase::SoftLimitDirection::kReverse};
    constexpr auto kRightMotorSoftLimitDirection {rev::CANSparkBase::SoftLimitDirection::kReverse};
    const bool kLeftMotorSoftLimitEnabled = true;
    const bool kRightMotorSoftLimitEnabled = true;
    constexpr double kLeftMotorSoftLimitValue {2.0};
    constexpr double kRightMotorSoftLimitValue {2.0};
}