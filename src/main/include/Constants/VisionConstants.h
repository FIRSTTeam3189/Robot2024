#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <iostream>

namespace VisionConstants {
    constexpr wpi::array<double, 3> kEncoderTrustCoefficients {0.1, 0.1, 0.1};
    constexpr wpi::array<double, 3> kVisionTrustCoefficients {0.5, 0.5, 0.5};
    constexpr double kVisionStdDevPerMeter {0.1};
    constexpr units::meter_t kCameraXOffset {0.308_m};
    constexpr units::meter_t kCameraYOffset {0.1778_m};
    constexpr units::meter_t kCameraZOffset {0.0_m};
    constexpr units::radian_t kCameraYawOffset {0.0_rad};
    constexpr bool kShouldUseVision {true};

    constexpr int kBaudRate {115200};
    constexpr int kBufferSize {1024};
    
    // Tag poses in order from 1 to 16
    const std::vector<frc::Pose3d> kTagPoses {
        frc::Pose3d{15.079_m, 0.246_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 120.0_deg}},
        frc::Pose3d{16.185_m, 0.884_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 120.0_deg}},
        frc::Pose3d{16.579_m, 4.983_m, 1.451_m, frc::Rotation3d{0.0_deg, 0.0_deg, 180.0_deg}},
        frc::Pose3d{16.579_m, 5.548_m, 1.451_m, frc::Rotation3d{0.0_deg, 0.0_deg, 180.0_deg}},
        frc::Pose3d{14.701_m, 8.204_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 270.0_deg}},
        frc::Pose3d{1.842_m, 8.204_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 270.0_deg}},
        frc::Pose3d{-0.038_m, 5.548_m, 1.451_m, frc::Rotation3d{0.0_deg, 0.0_deg, 0.0_deg}},
        frc::Pose3d{-0.038_m, 4.983_m, 1.451_m, frc::Rotation3d{0.0_deg, 0.0_deg, 0.0_deg}},
        frc::Pose3d{0.356_m, 0.884_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 60.0_deg}},
        frc::Pose3d{1.462_m, 0.246_m, 1.356_m, frc::Rotation3d{0.0_deg, 0.0_deg, 60.0_deg}},
        frc::Pose3d{11.905_m, 3.713_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 300.0_deg}},
        frc::Pose3d{11.905_m, 4.498_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 60.0_deg}},
        frc::Pose3d{11.220_m, 4.105_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 60.0_deg}},
        frc::Pose3d{5.321_m, 4.105_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 0.0_deg}},
        frc::Pose3d{4.641_m, 4.498_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 120.0_deg}},
        frc::Pose3d{4.641_m, 3.713_m, 1.321_m, frc::Rotation3d{0.0_deg, 0.0_deg, 240.0_deg}}
    };

    // Vision Sync bytes, in char format
    constexpr std::array kSyncBytes {'\x1a', '\xcf', '\xfc', '\x1d'};
}