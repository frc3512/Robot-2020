// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoNavBarrelRacing() {
    // Initial Pose - Robot positioned near the D1 marker in the Start & Finish
    // Zone
    const frc::Pose2d kInitialPose{1.08_m, 2.286_m, units::radian_t{0}};

    // D5 Entrance - Robot leaves the Stat & Finish Zone and begins the D5
    // Marker Loop
    const frc::Pose2d kD5Entrance{3.016_m - 8_in, 2.427_m, units::radian_t{0}};

    // D5 Poses - Robot loops around the D5 Marker
    const frc::Pose2d kD5Loop1{4.493_m - 5_in, 1.365_m,
                               units::radian_t{3 * wpi::numbers::pi / 2}};
    const frc::Translation2d kD5Loop2{3.256_m - 1_ft, 1.529_m - 1_ft};
    const frc::Translation2d kD5Loop3{3.256_m - 5_in, 1.529_m + 3_ft};

    // B8 Entrance Pose - Robot exits the D5 Loop and enters the B8 Loop
    const frc::Pose2d kB8Entrance{5.445_m - 5_in, 2.34_m,
                                  units::radian_t{wpi::numbers::pi / 18}};

    // B8 Poses - Robot loops around the B8 Marker
    const frc::Pose2d kB8Loop1{6.863_m - 5_in, 3.25_m,
                               units::radian_t{wpi::numbers::pi / 2}};
    const frc::Pose2d kB8Loop2{5.538_m - 1_ft, 3.029_m + 8_in,
                               units::radian_t{3 * wpi::numbers::pi / 2}};

    // D10 Entrance Pose - Robot exits B8 loop and enters the D10 loop
    const frc::Pose2d kD10Entrance{6.595_m - 1_ft, 1.289_m,
                                   units::radian_t{7 * wpi::numbers::pi / 4}};

    // D10 Poses - Robot loops around the D10 Marker
    const frc::Pose2d kD10Loop1{7.791_m - 1_ft, 0.98_m,
                                units::radian_t{wpi::numbers::pi / 9}};
    const frc::Pose2d kD10Loop2{8.527_m - 1_ft, 1.756_m + 8_in,
                                units::radian_t{wpi::numbers::pi / 2}};
    const frc::Translation2d kD10Loop3{7.114_m - 1_ft, 2.276_m};

    // Finish Approach Pose - Robot exists the D10 Marker loop and returns back
    // to the Start & Finish Zone
    const frc::Pose2d kFinishApproach{4.318_m - 1_ft, 2.62_m,
                                      units::radian_t{wpi::numbers::pi}};

    // End Pose - Robot positioned near the B1 marker in the Start & Finish Zone
    const frc::Pose2d kEndPose{0.35_m + 3_ft, 2.614_m,
                               units::radian_t{wpi::numbers::pi}};

    m_turret.SetControlMode(TurretController::ControlMode::kManual);
    m_drivetrain.Reset(kInitialPose);

    auto config1 =
        m_drivetrain.MakeTrajectoryConfig(0_mps, DrivetrainController::kMaxV);
    auto config2 = m_drivetrain.MakeTrajectoryConfig(
        DrivetrainController::kMaxV, DrivetrainController::kMaxV);
    auto config3 =
        m_drivetrain.MakeTrajectoryConfig(DrivetrainController::kMaxV, 0_mps);

    m_drivetrain.AddTrajectory({kInitialPose, kD5Entrance, kD5Loop1}, config1);
    m_drivetrain.AddTrajectory(kD5Loop1, {kD5Loop2, kD5Loop3}, kB8Entrance,
                               config2);
    m_drivetrain.AddTrajectory(
        {kB8Entrance, kB8Loop1, kB8Loop2, kD10Entrance, kD10Loop1, kD10Loop2},
        config2);
    m_drivetrain.AddTrajectory(kD10Loop2, {kD10Loop3}, kFinishApproach,
                               config2);
    m_drivetrain.AddTrajectory(kFinishApproach, {}, kEndPose, config3);

    if (!m_autonChooser.Suspend([=] { return m_drivetrain.AtGoal(); })) {
        return;
    }
}
}  // namespace frc3512
