#pragma once

#include <iostream>

namespace ClimberConstants{
    //FIND IDS - krishna 
    constexpr int kLeftServoID {0}; 
    constexpr int kRightServoID {1}; 
    constexpr int kRightMotorID {19};
    constexpr int kLefttMotorID {20};
    constexpr int kLimitSwitchPort {2};
    constexpr bool kInvertLeftMotor {false};
    constexpr bool kInvertRightMotor {false};
    constexpr double kExtendPower {0.5};
    constexpr double kExtendServoAngle {90.0};
    constexpr double kRetractPower {-0.5};
    constexpr double kRetractServoAngle {0.0};
    constexpr auto kIdleMode {rev::CANSparkMax::IdleMode::kBrake};
}