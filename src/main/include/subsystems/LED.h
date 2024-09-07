// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <vector>
#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/led/ColorFlowAnimation.h>
#include <ctre/phoenix/led/FireAnimation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <ctre/phoenix/led/RainbowAnimation.h>
#include <ctre/phoenix/led/RgbFadeAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>
#include <ctre/phoenix/led/TwinkleAnimation.h>
#include <ctre/phoenix/led/TwinkleOffAnimation.h>
#include "Constants/LEDConstants.h"

enum class LEDAnimationType { Clear, ColorFlow, Fire, Larson, Rainbow, RGBFade, SingleFade, Strobe, Twinkle, TwinkleOff }; 
enum class LEDSection { All, Candle, AllLEDMatrix, LEDMatrix1, LEDMatrix2, Row0, Row1, Row2, Row3, Row4, Row5, Row6, Row7, Row8, Row9, Row10, Row11, Row12, Row13, Row14, Row15};
//static std::map<std::string, std::vector<std::vector<bool>>> s_LEDDictionary;
class LED : public frc2::SubsystemBase {
 public:
  LED();
  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

 static constexpr int kLength = 200;
 
 frc::AddressableLED m_led{9};
 std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  // Reuse the buffer
  // Store what the last hue of the first pixel is
  int firstPixelHue = 0;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

};
