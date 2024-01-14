// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LED.h"
#include "Constants.h"

LED::LED(): m_candleControl(LedConstants::kCandleID), m_ledSections(), m_animation(NULL), m_shouldStartup(true), m_startupRunning(false){
     m_candleControl.ConfigAllSettings(m_candleConfig);
}   

// This method will be called once per scheduler run
void LED::Periodic() {

}

void LED::SetColor(int r, int g, int b,LEDSection section){
    m_candleControl.SetLEDs(r, g , b, 0, m_ledSections[section].first, m_ledSections[section].second - m_ledSections[section].first);
}


void LED::ClearColor(LEDSection section) {
    // Clears the LEDs in the specified section of lights
    m_candleControl.SetLEDs(0, 0, 0, 0,  m_ledSections[section].first, m_ledSections[section].second - m_ledSections[section].first);
}

void LED::SetAnimation(LEDAnimationType newAnimation, LEDSection section, int r, int g, int b, double speed, bool reverse, int animSlot) {
    auto len = m_ledSections[section].second - m_ledSections[section].first;
    // Sets the Color Flow animation reversed based on reversed bool
    auto cfDir = ColorFlowAnimation::Direction::Forward;
     switch (newAnimation) {
    case LEDAnimationType::ColorFlow:
        m_animation = new ColorFlowAnimation(r, g, b, 0, speed, len, cfDir, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Fire:
        m_animation = new FireAnimation(m_candleConfig.brightnessScalar, speed, len, 0.7, 0.5, reverse, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Larson:
        m_animation = new LarsonAnimation(r, g, b, 0, speed, len, LarsonAnimation::BounceMode::Front, 20, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Rainbow:
        m_animation = new RainbowAnimation(m_candleConfig.brightnessScalar, speed, len, reverse, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::RGBFade:
        m_animation = new RgbFadeAnimation(m_candleConfig.brightnessScalar, speed, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::SingleFade:
        m_animation = new SingleFadeAnimation(r, g, b, 0, speed, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Strobe:
        m_animation = new StrobeAnimation(r, g, b, 0, speed, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Twinkle:
        m_animation = new TwinkleAnimation(r, g, b, 0, speed, len, TwinkleAnimation::TwinklePercent::Percent100, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::TwinkleOff:
        m_animation = new TwinkleOffAnimation(r, g, b, 0, speed, len, TwinkleOffAnimation::TwinkleOffPercent::Percent100, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Clear:
    // Clears all the animations in the available animation slots
        m_candleControl.ClearAnimation(0);
        m_candleControl.ClearAnimation(1);
        m_candleControl.ClearAnimation(2);
        m_candleControl.ClearAnimation(3);
        m_candleControl.ClearAnimation(4);
        m_candleControl.ClearAnimation(5);
        m_candleControl.ClearAnimation(6);
        m_candleControl.ClearAnimation(7);
        m_candleControl.ClearAnimation(8);
        break;
}
}

void LED::ClearAll(LEDSection section){
    m_candleControl.SetLEDs(0, 0, 0, 0,  m_ledSections[section].first, m_ledSections[section].second - m_ledSections[section].first);
}

void LED::StartingAnimation(){
if (!m_startupRunning) {
        SetAnimation(LEDAnimationType::Clear);
        m_startupRunning = true;
        m_Timer.Start();
        SetAnimation(LEDAnimationType::ColorFlow, LEDSection::All, 255, 255, 0, 0.5, true);
        SetAnimation(LEDAnimationType::ColorFlow, LEDSection::All, 0, 0, 255, 0.5, false, 1);
    }
    if (m_Timer.Get() > 3.0_s) {
        m_shouldStartup = false;
        SetAnimation(LEDAnimationType::Clear);
        m_Timer.Stop();
    }
}