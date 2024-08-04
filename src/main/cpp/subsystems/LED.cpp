// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// doesn't really work since we dont have led's on the robot
// if working, it will take input string and convert it to LED pixels

#include "subsystems/LED.h"
static std::map<std::string, std::vector<std::vector<bool>>> s_LEDDictionary = {
    {"a", {{false, true, true, true, true, false}, {true, false, false, false, false, true}, {true, false, false, false, false, true}, {true, true, true, true, true, true}, {true, false, false, false, false, true}, {true, false, false, false, false, true}, {true, false, false, false, false, true}, {true, false, false, false, false, true}}},
};

LED::LED(): m_candleControl(LedConstants::kCandleID), m_ledSections(), m_animation(NULL), m_shouldStartup(true), m_startupRunning(false), m_runString(false), m_shouldRunString(false), m_lastEnableState(false){
    m_candleControl.ConfigAllSettings(m_candleConfig);

    m_ledSections[LEDSection::All] = {0, 520};

    // Candle length 8
    m_ledSections[LEDSection::Candle] = {0, 8};
    m_ledSections[LEDSection::AllLEDMatrix] = {8, 520};
    m_ledSections[LEDSection::LEDMatrix1] = {8, 264};
    m_ledSections[LEDSection::LEDMatrix2] = {264, 520};

    // Rows length 32 each
    
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
    // std::cout << "LED constructing\n";
}   

// This method will be called once per scheduler run
void LED::Periodic() {
    if (m_shouldStartup) {
        Search("aaaaa", 3);
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
        if (m_runString) {
            ClearColor(LEDSection::All);
            DisplayString();
            m_curStrIndex -= 1;
            if (m_curStrIndex == -1) {
                m_curStrIndex = 0;
                if (m_arr.size() == 0) {
                    m_runString = false;
                }
            } 
        }
    }    
}

void LED::SetSectionColor(int r, int g, int b,LEDSection section){
    m_candleControl.SetLEDs(r, g , b, 0, m_ledSections[section].first, m_ledSections[section].second - m_ledSections[section].first);
}

void LED::SetRowColor(int r, int g, int b, std::pair<uint8_t, uint8_t> section){
    m_candleControl.SetLEDs(r, g, b, 0, section.first, section.second - section.first);
}

// Columns and row counts start with 1 not 0
void LED::SetColumnColor(int r, int g, int b, int col, int start, int end){
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
        
        // SetAnimation(LEDAnimationType::ColorFlow, LEDSection::All, 255, 255, 0, 0.5, true);
        // SetAnimation(LEDAnimationType::ColorFlow, LEDSection::All, 0, 0, 255, 0.5, false, 1);
    }
    if (m_Timer.Get() > 3.0_s) {
        m_shouldStartup = false;
        SetAnimation(LEDAnimationType::Clear);
        m_Timer.Stop();
    }
}

void LED::Search(std::string str, int length){
    /* std::vector<int> col = {};
    std::vector<std::vector<int>> row = {};
    int pixel;
    for (int i = 0; i < length; i++){
        std::string letter = str.substr(i, i+1);
        for (size_t j = 0; j < s_LEDDictionary[letter].size(); i++) {
            for (size_t x = 0; x < s_LEDDictionary[letter][j].size(); x++) {
                pixel = s_LEDDictionary[letter][j][x];
                col.push_back(pixel);
            }
            row.push_back(col);
        }
        m_arr.push_back(row);
        row = {};
        col = {};
        //i[0] is first letter , i[1] is second letter....
    }
    m_runString = true;
    m_shouldRunString = true;
    m_curStrIndex = 31; */
    std::vector<bool> col = {};
    std::vector<std::vector<bool>> row = {};
    for (int i = 0; i < length; i++){
        std::string letter = str.substr(i, i+1);
        for (size_t j = 0; j < s_LEDDictionary[letter].size(); i++) {
            for (size_t x = 0; x < s_LEDDictionary[letter][j].size(); x++) {
                col.push_back(s_LEDDictionary[letter][j][x]);
            }
            row.push_back(col);
        }
        m_arr.push_back(row);
        row = {};
        col = {};
        //i[0] is first letter , i[1] is second letter....
    }
    m_runString = true;
    m_shouldRunString = true;
    m_curStrIndex = 31;
}

void LED::SetMap() {
    /* if (m_shouldRunString) {
        m_Timer.Start();
        m_shouldRunString = false;
    } else if (m_Timer.Get() > 0.5_s) {
        std::vector<std::vector<std::vector<int>>>::iterator letter_it;
        std::vector<std::vector<int>>::iterator row_it;
        std::vector<int>::iterator col_it;
        for (size_t i = 0; i < m_arr.size(); i++) {
            letter_it = m_arr.begin() + i;
            for (size_t j = letter_it; j < m_arr[i].size(); j++) {
                row_it = m_arr[i].begin() + j;
                for (size_t x = 0; x < m_arr[i][j].size(); x++) {
                    col_it = m_arr[i][j].begin() + x;
                    col_it = m_arr[i][j].begin() + x;
                    int index = (m_curStrIndex + m_arr[i][j][x] - 1);
                    if (index > 31) {
                        index = 0;
                    } else if (index < 0) {
                        m_arr[i][j].erase(col_it);
                    } else {
                        index = index + (32*j) + 8;
                        SetRowColor(0, 0, 255, {index, index + 1});
                    }
                }
                if (m_arr[i][j].size() == 0) {
                    m_arr[i].erase(row_it);
                }
            }
            if (m_arr[i].size() == 0) {
                m_arr.erase(letter_it);
            }
        }
        m_Timer.Stop();
        m_shouldRunString = true;
    } */
    if (m_shouldRunString) {
        m_Timer.Start();
        m_shouldRunString = false;
    } else if (m_Timer.Get() > 0.5_s) {
        for (size_t i = 0; i < m_arr.size(); i++) {
            for (int j = 0; j < 6; j++) {
                for (int x = 0; x < 8; i++) {
                    m_LEDMap[j][31] = m_arr[i][x][j];
                }
                /* for (size_t x = 0; x < m_arr[i][j].size(); x++) {
                    
                    if (m_arr[i][j][x]) {
                        int index = x + (32*j) + 8;
                        SetRowColor(0, 0, 255, {index, index + 1});
                    }
                } */
            }
        }
        m_Timer.Stop();
        m_shouldRunString = true;
        DisplayString();
    }
}

void LED::DisplayString() {
    for (int j = 0; j < 8; j++) {
        for (int i = 0; i < 32; i++) {
            if (m_LEDMap[j][i+1]) {
                SetRowColor(0, 0, 255, {i+8, i+9});
            }
        }
    }
    for (int j = 0; j < 8; j++) {
        for (int i = 1; i < 31; i++) {
            m_LEDMap[j][i-1] = m_LEDMap[j][i];
        }
    }
}

/* std::map<std::string, std::vector<std::vector<int>>> s_LEDDictionary = {
    {"a", {{2, 3, 4, 5}, {1, 6}, {1, 6}, {1, 2, 3, 4, 5, 6}, {1, 6}, {1, 6}, {1, 6}, {1, 6}}},
    {"b", {{1, 2, 3, 4, 5}, {1, 6}, {1, 6}, {1, 2, 3, 4, 5}, {1, 6}, {1, 6}, {1, 6}, {1, 2, 3, 4, 5, 6}}},
    {"c", {{1, 2, 3, 4, 5, 6}, {1}, {1}, {1}, {1}, {1}, {1}, {1, 2, 3, 4, 5, 6}}},
    {"d", {{1, 2, 3, 4}, {1, 4}, {1, 6}, {1, 6}, {1, 6}, {1, 6}, {1, 5}, {1, 2, 3, 4}}},
    {"e", {{1, 2, 3, 4, 5, 6}, {1}, {1}, {1}, {1, 2, 3, 4, 5}, {1}, {1}, {1, 2, 3, 4, 5, 6}}},
    {"f", {{1, 2, 3, 4, 5, 6}, {1}, {1}, {1}, {1, 2, 3, 4, 5}, {1}, {1}, {1}}},
    {"g", {{1, 2, 3, 4, 5, 6}, {1}, {1}, {1}, {1, 3, 4, 5, 6}, {1, 6}, {1, 6}, {1, 2, 3, 4, 5, 6}}},
    {"h", {{1, 6}, {1, 6}, {1, 6}, {1, 6}, {1, 2, 3, 4, 5, 6}, {1, 6}, {1, 6}, {1, 6}}},
    {"i", {{1, 2, 3, 4, 5, 6}, {3, 4}, {3, 4}, {3, 4}, {3, 4}, {3, 4}, {3, 4}, {1, 2, 3, 4, 5, 6}}},
    {"j", {{6}, {6}, {6}, {6}, {6}, {6}, {1, 6}, {2, 3, 4, 5}}},
    {"k", {{1, 6}, {1, 5}, {1, 4}, {1, 2, 3}, {1, 3, 4}, {1, 5}, {1, 6}, {1, 6}}},
    {"l", {{1}, {1}, {1}, {1}, {1}, {1}, {1}, {1, 2, 3, 4, 5, 6}}},
    {"m", {{1, 6}, {1, 2, 5, 6}, {1, 3, 4, 6}, {1, 6}, {1, 6}, {1, 6}, {1, 6}, {1, 6}}},
    {"n", {{1, 6}, {1, 6}, {1, 2, 6}, {1, 3, 6}, {1, 4, 6}, {1, 5, 6}, {1, 6}, {1, 6}}},
    {"o", {{2, 3, 4, 5}, {1, 2, 5, 6}, {1, 6}, {1, 6}, {1, 6}, {1, 6}, {1, 2, 5, 6}, {2, 3, 4, 5}}},
    {"p", {{2, 3, 4, 5}, {2, 5}, {2, 5}, {2, 3, 4, 5, 6}, {2}, {2}, {2}, {2}}},
    {"q", {{2, 3, 4, 5}, {1, 6}, {1, 6}, {1, 6}, {1, 4, 6}, {1, 4, 6}, {2, 3, 4, 5}, {6}}},
    {"r", {{1, 2, 3, 4, 5}, {1, 5}, {1, 5}, {1,2,3,4,5}, {1,2,3}, {1,3,4}, {1, 4, 5}}},
    {"s", {{1,2,3,4,5,6}, {1}, {1}, {1}, {1,2,3,4,5,6}, {6}, {6}, {1,2,3,4,5,6}}},
    {"t", {{1,2,3,4,5,6}, {1,2,3,4,5,6}, {3,4}, {3,4}, {3,4}, {3,4}, {3,4}, {3,4}}},
    {"u", {{1,6}, {1,6}, {1,6}, {1,6}, {1,6}, {1,6}, {1,6}, {1,2,3,4,5,6}}},
    {"v", {{1,6}, {1,6}, {1,6}, {1,6}, {2,5}, {2,5}, {2,5}, {3,4}}},
    {"w", {{2,4,6}, {2,4,6}, {2,4,6}, {2,4,6}, {2,4,6}, {2,4,6}, {2,4,6}, {2,3,4,5,6}}},
    {"x", {{1, 6}, {2, 5}, {2, 5}, {3, 4}, {3, 4}, {2, 5}, {2, 5}, {1, 6}}},
    {"y", {{1,6}, {1,6}, {1,2,5,6}, {2,3,4,5}, {3,4}, {3,4}, {3,4}, {3,4}}},
    {"z", {{1, 2, 3, 4, 5, 6}, {6}, {5}, {4}, {3}, {2}, {1}, {1, 2, 3, 4, 5, 6}}},
    // NUMBERS
    {"0", {{2, 3, 4, 5}, {1, 6}, {1, 6}, {1, 6}, {1, 6}, {1, 6}, {1, 6}, {2, 3, 4, 5}}},
    {"1", {{4}, {2, 3, 4}, {2, 4}, {4}, {4}, {4}, {4}, {1, 2, 3, 4, 5}}},
    {"2", {{1, 2, 3, 4, 5, 6}, {6}, {6}, {6}, {1, 2, 3, 4, 5, 6}, {1}, {1}, {1, 2, 3, 4, 5, 6}}},
    {"3", {{2, 3, 4, 5}, {1, 6}, {6}, {6}, {2, 3, 4, 5}, {6}, {1, 6}, {2, 3, 4, 5}}},
    {"4", {{1, 5}, {1, 5}, {1, 5}, {1, 2, 3, 4, 5, 6}, {5}, {5}, {5}, {5}}},
    {"5", {{1, 2, 3, 4, 5}, {1}, {1}, {1, 2, 3, 4, 5}, {6}, {6}, {6}, {1, 2, 3, 4, 5}}},
    {"6", {{2, 3, 4, 5}, {1, 6}, {1}, {1}, {1, 2, 3, 4, 5}, {1, 6}, {1, 6}, {2, 3, 4, 5}}},
    {"7", {{1, 2, 3, 4, 5, 6}, {6}, {6}, {6}, {6}, {6}, {6}, {6}}},
    {"8", {{2, 3, 4, 5}, {1, 6}, {1, 6}, {1, 6}, {2, 3, 4, 5}, {1, 6}, {1, 6}, {2, 3, 4, 5}}},
    {"9", {{1, 2, 3, 4, 5, 6}, {1, 6}, {1, 6}, {1, 2, 3, 4, 5, 6}, {6}, {6}, {6}, {6}}},
}; */