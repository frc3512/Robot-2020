// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoNavBarrelRacing() {
    // Initial Pose - Robot positioned near the D1 marker in the Start & Finish
    // Zone
    const frc::Pose2d kInitialPose{0.40_m, 2.077_m, units::radian_t{0}};

    // D5 Entrance - Robot leaves the Stat & Finish Zone and begins the D5
    // Marker Loop
    const frc::Pose2d kD5Entrance{3.016_m, 2.427_m, units::radian_t{0}};

    // D5 Poses - Robot loops around the D5 Marker
    const frc::Pose2d kD5Loop1{4.493_m, 1.365_m,
                               units::radian_t{3 * wpi::math::pi / 2}};
    const frc::Pose2d kD5Loop2{3.256_m, 1.529_m,
                               units::radian_t{wpi::math::pi / 3}};

    // B8 Entrance Pose - Robot exits the D5 Loop and enters the B8 Loop
    const frc::Pose2d kB8Entrance{5.445_m, 2.34_m,
                                  units::radian_t{wpi::math::pi / 18}};

    // B8 Poses - Robot loops around the B8 Marker
    const frc::Pose2d kB8Loop1{6.863_m, 3.25_m,
                               units::radian_t{wpi::math::pi / 2}};
    const frc::Pose2d kB8Loop2{5.538_m, 3.029_m,
                               units::radian_t{3 * wpi::math::pi / 2}};

    // D10 Entrance Pose - Robot exits B8 loop and enters the D10 loop
    const frc::Pose2d kD10Entrance{6.595_m, 1.289_m,
                                   units::radian_t{7 * wpi::math::pi / 4}};

    // D10 Poses - Robot loops around the D10 Marker
    const frc::Pose2d kD10Loop1{7.791_m, 0.98_m,
                                units::radian_t{wpi::math::pi / 9}};
    const frc::Pose2d kD10Loop2{8.527_m, 1.756_m,
                                units::radian_t{wpi::math::pi / 2}};

    const frc::Pose2d kD10Loop3{7.114_m, 2.276_m,
                                units::radian_t{8 * wpi::math::pi / 9}};

    // Finish Approach Pose - Robot exists the D10 Marker loop and returns back
    // to the Start & Finish Zone
    const frc::Pose2d kFinishApproach{4.318_m, 2.62_m,
                                      units::radian_t{wpi::math::pi}};

    // End Pose - Robot positioned near the B1 marker in the Start & Finish Zone
    const frc::Pose2d kEndPose{0.35_m, 2.614_m, units::radian_t{wpi::math::pi}};

    m_turret.SetControlMode(TurretController::ControlMode::kManual);
    m_drivetrain.Reset(kInitialPose);

    m_drivetrain.AddTrajectory({kInitialPose, kD5Entrance, kD5Loop1, kD5Loop2,
                                kB8Entrance, kB8Loop1, kB8Loop2, kD10Entrance,
                                kD10Loop1, kD10Loop2, kD10Loop3,
                                kFinishApproach, kEndPose});

    while (!m_drivetrain.AtGoal()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }
}
}  // namespace frc3512
