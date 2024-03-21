#pragma once

#include <iostream>

namespace ClimberConstants{
    //FIND IDS - krishna 
    constexpr int kLeftServoID {3}; 
    constexpr int kRightServoID {0}; 
    constexpr int kRightMotorID {19};
    constexpr int kLeftMotorID {20};
    constexpr int kLimitSwitchPort {4};
    constexpr bool kInvertLeftMotor {false};
    constexpr bool kInvertRightMotor {false};
    constexpr double kExtendPower {0.5};
    constexpr double kBothExtendPower {0.75};
    constexpr double kLeftExtendServoAngle {0.75};
    constexpr double kRightExtendServoAngle {0.0};
    constexpr double kRetractPower {-0.5};
    constexpr double kBothRetractPower {-1.0};
    constexpr double kLeftRetractServoAngle {0.0};
    constexpr double kRightRetractServoAngle {0.5};
    constexpr auto kIdleMode {rev::CANSparkMax::IdleMode::kBrake};
}