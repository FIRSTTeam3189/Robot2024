// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <vector>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
// #include <ctre/Phoenix.h>
// #include <ctre/phoenix/led/ColorFlowAnimation.h>
// #include <ctre/phoenix/led/FireAnimation.h>
// #include <ctre/phoenix/led/LarsonAnimation.h>
// #include <ctre/phoenix/led/RainbowAnimation.h>
// #include <ctre/phoenix/led/RgbFadeAnimation.h>
// #include <ctre/phoenix/led/SingleFadeAnimation.h>
// #include <ctre/phoenix/led/StrobeAnimation.h>
// #include <ctre/phoenix/led/TwinkleAnimation.h>
// #include <ctre/phoenix/led/TwinkleOffAnimation.h>
// #include "Constants/LEDConstants.h"

// enum class LEDAnimationType { Clear, ColorFlow, Fire, Larson, Rainbow, RGBFade, SingleFade, Strobe, Twinkle, TwinkleOff }; 
// enum class LEDSection { All, Candle, AllLEDMatrix, LEDMatrix1, LEDMatrix2, Row0, Row1, Row2, Row3, Row4, Row5, Row6, Row7, Row8, Row9, Row10, Row11, Row12, Row13, Row14, Row15};
// //static std::map<std::string, std::vector<std::vector<bool>>> s_LEDDictionary;
// class LED : public frc2::SubsystemBase {
//  public:
//   LED();
    
//   void SetSectionColor(int r, int g, int b, LEDSection section = LEDSection::All);
//   void SetRowColor(int r, int g, int b, std::pair<uint8_t, uint8_t> section = {0, 520});
//   void SetColumnColor(int r, int g, int b, int col, int start, int end);
//   void ClearColor(LEDSection section = LEDSection::All);
//   void SetAnimation(LEDAnimationType animation, LEDSection section = LEDSection::All, int r = 0, int g = 0, int b = 0, double speed = 0.7, bool reverse = false, int animSlot = 0);
//   void StartingAnimation();
//   void CBAnimation();
//   void ClearAll(LEDSection section);
//   void Search(std::string str, int length);
//   void SetMap();
//   void DisplayString();

//   /**
//    * Will be called periodically whenever the CommandScheduler runs.
//    */
//   void Periodic() override;

//  private:
 
//   // Components (e.g. motor controllers and sensors) should generally be
//   // declared private and exposed only through public methods.
// ctre::phoenix::led::CANdle m_candleControl;
// std::map<LEDSection, std::pair<uint8_t, uint8_t>> m_ledSections;
// ctre::phoenix::led::Animation *m_animation;
// bool m_shouldStartup;
// bool m_startupRunning;
// bool m_runString;
// bool m_shouldRunString;
// bool m_lastEnableState;
// int m_curStrIndex = 0;
// CANdleConfiguration m_candleConfig;
// frc::Timer m_Timer{};
// //static std::map<std::string, std::vector<std::vector<int>>> s_LEDDictionary;

// //std::vector<std::vector<std::vector<int>>> m_arr {};
// std::vector<std::vector<std::vector<bool>>> m_arr {};
// bool m_LEDMap[8][32];
// };
