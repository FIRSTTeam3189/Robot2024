#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#define Pi 3.14159265358979323846

enum class ActiveDriveController { OfficialDriver, TestControls };

namespace SwerveDriveConstants {
    constexpr ActiveDriveController kActiveController { ActiveDriveController::OfficialDriver };

    constexpr int kGyroID {13};
    constexpr double kRadiansToDegreesMultiplier {180.0 / Pi};

    // Coordinate plane distance in meters to each swerve drive
    // This has x-positive as forward, y-positive as left
    constexpr auto kXDistanceFromCenter {0.282575_m};
    constexpr auto kYDistanceFromCenter {0.282575_m};
    constexpr auto kWheelbase {0.282575_m * 2.0};
    constexpr auto kTrackwidth {0.282575_m * 2.0};

    constexpr units::degree_t kGyroMountPoseYaw {0.0_deg};

    constexpr auto kAmpAlignTarget = 90.0_deg;
    constexpr auto kBlueSourceAlignTarget = 120.0_deg;
    constexpr auto kRedSourceAlignTarget = 60.0_deg;

    constexpr auto kRefreshRate = units::frequency::hertz_t{100};

    static frc::SwerveDriveKinematics<4> kKinematics {
        frc::Translation2d{+SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{+SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{-SwerveDriveConstants::kXDistanceFromCenter, +SwerveDriveConstants::kYDistanceFromCenter},
        frc::Translation2d{-SwerveDriveConstants::kXDistanceFromCenter, -SwerveDriveConstants::kYDistanceFromCenter}
    };
    constexpr auto kMaxSpeed {4.0_mps};
    constexpr auto kMaxAcceleration {5.0_mps_sq};
    constexpr units::radians_per_second_t kMaxAngularVelocity {4.0 * Pi};
    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration {2.0 * Pi};
    constexpr double kDecelerationLimit {-3.5}; // meters per second

    // SysID robot characterization values -- **varies by robot**
    constexpr double kSRot {0.125}; // 0.408_V
    // constexpr auto kv {3.206 * 1_V * 1_s / 1_m};
    // constexpr auto ka {3.409 * 1_V * 1_s * 1_s / 1_m};

    // These are for robot rotation, not wheel rotation
    constexpr double kPRot {0.005};
    constexpr double kIRot {0.0}; // constexpr double kIRot {0.00025};
    constexpr double kDRot {0.0};

    constexpr double kSlowModeDriveMultiplier {0.35}; // 0.5
    constexpr double kSwerveRotationTolerance {0.5};

    constexpr double kDecelerationDeadband {0.05};
    constexpr bool kShouldDecelerate = false;
}
