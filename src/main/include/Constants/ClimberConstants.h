#pragma once

#include <iostream>

namespace ClimberConstants{
    //FIND IDS - krishna 
    constexpr int kLeftServoID {0}; 
    constexpr int kRightServoID {1}; 
    constexpr int kRightMotorID {19};
    constexpr int kLefttMotorID {20};
    constexpr bool kInvertLeftMotor {false};
    constexpr bool kInvertRightMotor {false};
    constexpr double kExtendMotorSpeed {0.5};
    constexpr double kExtendServoAngle {90.0};
    constexpr double kRetractMotorSpeed {-0.5};
    constexpr double kRetractServoAngle {0.0};
}