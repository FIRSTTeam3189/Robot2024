// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LED.h"

LED::LED(): m_candleControl(LedConstants::kCandleID), m_ledSections(), m_animation(NULL), m_shouldStartup(true), m_startupRunning(false){
     m_candleControl.ConfigAllSettings(m_candleConfig);

    m_ledSections[LEDSection::All] = {0, 520};

    // Candle length 8
    m_ledSections[LEDSection::Candle] = {0, 8};

    // Rows length 32 each
    m_ledSections[LEDSection::LEDMatrix] = {8, 520};
    m_ledSections[LEDSection::Row0] = {8, 40};
    m_ledSections[LEDSection::Row1] = {40, 72};
    m_ledSections[LEDSection::Row2] = {72, 104};
    m_ledSections[LEDSection::Row3] = {104, 136};
    m_ledSections[LEDSection::Row4] = {136, 168};
    m_ledSections[LEDSection::Row5] = {168, 200};
    m_ledSections[LEDSection::Row6] = {200, 232};
    m_ledSections[LEDSection::Row7] = {232, 264};
    m_ledSections[LEDSection::Row8] = {264, 296};
    m_ledSections[LEDSection::Row9] = {296, 328};
    m_ledSections[LEDSection::Row10] = {328, 360};
    m_ledSections[LEDSection::Row11] = {360, 392};
    m_ledSections[LEDSection::Row12] = {392, 424};
    m_ledSections[LEDSection::Row13] = {424, 456};
    m_ledSections[LEDSection::Row14] = {456, 488};
    m_ledSections[LEDSection::Row15] = {488, 520};
}   

// This method will be called once per scheduler run
void LED::Periodic() {
    if (m_shouldStartup) {
        StartingAnimation();
    }
    else if (!frc::SmartDashboard::GetBoolean("Enabled", false)) {
        if (m_lastEnableState == true) {
            SetAnimation(LEDAnimationType::Clear);
        }
        SetAnimation(LEDAnimationType::Rainbow, LEDSection::All, 0, 120, 255);
    }
    else {
        if (m_lastEnableState == false) {
            SetAnimation(LEDAnimationType::Clear);
            m_lastEnableState = true;
        }
        

    }    
}

void LED::SetColor(int r, int g, int b,LEDSection section){
    m_candleControl.SetLEDs(r, g , b, 0, m_ledSections[section].first, m_ledSections[section].second - m_ledSections[section].first);
}

void LED::SetRowColor(int r, int g, int b, std::pair<uint8_t, uint8_t> section){
    m_candleControl.SetLEDs(r, g, b, 0, section.first, section.second - section.first);
}

// Columns and row counts start with 1 not 0
void LED::SetColumnColor(int r, int g, int b, uint_8 col, int start, int end){
    uint8_t count = 0;
    for (int i = start - 1; i <= end - 1; i++){
        count = (32*i) + 7 + col;
        m_candleControl.SetLEDs(r, g, b, 0, count, count + 1);
    }
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

void LED::CBAnimation() {
    SetAnimation(LEDAnimationType::Clear);
    SetColumnColor(0, 0, 255, 7, 1, 16);

    SetColumnColor(0, 0, 255, 15, 1, 16);

    SetColumnColor(0, 0, 255, 18, 1, 4);
    SetColumnColor(0, 0, 255, 18, 5, 8);
    SetColumnColor(0, 0, 255, 23, 1, 4);
    SetColumnColor(0, 0, 255, 23, 5, 8);
    
    SetColumnColor(0, 0, 255, 26, 1, 4);
    SetColumnColor(0, 0, 255, 31, 1, 8);
    
    SetRowColor(0, 0, 255, {9, 15});
    SetRowColor(0, 0, 255, {109, 111});
    SetRowColor(0, 0, 255, {141, 143});
    SetRowColor(0, 0, 255, {233, 239});

    SetRowColor(0, 0, 255, {26, 30});
    SetRowColor(0, 0, 255, {122, 126});
    SetRowColor(0, 0, 255, {154, 158});
    SetRowColor(0, 0, 255, {250, 254});

    SetRowColor(0, 0, 255, {34, 38});
    SetRowColor(0, 0, 255, {130, 134});

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