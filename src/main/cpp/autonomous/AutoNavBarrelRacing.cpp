// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <units/math.h>

#include "Robot.hpp"

std::vector<frc::Translation2d> MakeSemicircle(frc::Translation2d center,
                                               frc::Pose2d start,
                                               frc::Rotation2d exitHeading) {
    auto r = start.Translation() - center;
    frc::Pose2d end{center + r.RotateBy(exitHeading - start.Rotation()),
                    exitHeading};

    fmt::print(stderr, "|r|={}\n", r.Norm());
    fmt::print(stderr, "start pose={}, {}, {}\n", start.X(), start.Y(),
               start.Rotation().Radians());
    fmt::print(stderr, "end pose={}, {}, {}\n", end.X(), end.Y(),
               end.Rotation().Radians());

    auto mvec = start.Translation() - end.Translation();
    auto rvec = center - start.Translation();
    auto det = mvec.X() * rvec.Y() - mvec.Y() * rvec.X();

    auto headingChange = exitHeading.Radians() - start.Rotation().Radians();
    int steps;
    if (headingChange == 0_rad) {
        steps = 360_deg / 10_deg;
    } else if (det.to<double>() > 0.0) {
        steps = units::math::abs(headingChange) / 10_deg;
    } else {
        steps = units::math::abs(headingChange) / 10_deg;
    }
    fmt::print(stderr, "steps={}\n", steps);

    std::vector<frc::Translation2d> arc;
    for (int i = 0; i < steps; ++i) {
        if (det.to<double>() > 0.0) {
            arc.emplace_back(center + r.RotateBy(10_deg * i));
        } else {
            arc.emplace_back(center + r.RotateBy(-10_deg * i));
        }
    }

    arc.emplace_back(center + r.RotateBy(headingChange));

    fmt::print(stderr, "x,y\n");
    for (const auto& p : arc) {
        fmt::print(stderr, "{},{}\n", p.X().to<double>(), p.Y().to<double>());
    }

    return arc;
}

namespace frc3512 {
void Robot::AutoNavBarrelRacing() {
    // Initial Pose - Robot positioned near the D1 marker in the Start & Finish
    // Zone
    const frc::Pose2d kInitialPose{0.40_m, 2.077_m, 0_rad};

    // End Pose - Robot positioned near the B1 marker in the Start & Finish Zone
    const frc::Pose2d kEndPose{0.35_m, 2.614_m, 180_deg};

    m_turret.SetControlMode(TurretController::ControlMode::kManual);
    m_drivetrain.Reset(kInitialPose);

    std::vector<frc::Translation2d> path;

    fmt::print(stderr, "initialPose={},{}\n", kInitialPose.X().to<double>(),
               kInitialPose.Y().to<double>());

    const frc::Translation2d kD5{3.83_m, 1.507_m};
    auto circle1 = MakeSemicircle(
        kD5, {kD5 + frc::Translation2d{0_m, 0.6_m}, 0_rad}, 0_rad);
    path.insert(path.end(), circle1.begin(), circle1.end());

    const frc::Translation2d kB8{6.11_m, 3.036_m};
    auto circle2 = MakeSemicircle(
        kB8, {kB8 + frc::Translation2d{0_m, -0.6_m}, 45_deg}, 270_deg);
    path.insert(path.end(), circle2.begin(), circle2.end());

    const frc::Translation2d kD10{7.639_m, 1.498_m};
    auto circle3 = MakeSemicircle(
        kD10, {kD10 + frc::Translation2d{-0.6_m, 0_m}, 270_deg}, 180_deg);
    path.insert(path.end(), circle3.begin(), circle3.end());

    fmt::print(stderr, "endPose={},{}\n", kEndPose.X().to<double>(),
               kEndPose.Y().to<double>());

    m_drivetrain.AddTrajectory(kInitialPose, path, kEndPose);
    // m_drivetrain.AddTrajectory({kInitialPose, kD5Entrance, kD5Loop1,
    // kD5Loop2,
    //                             kB8Entrance, kB8Loop1, kB8Loop2,
    //                             kD10Entrance, kD10Loop1, kD10Loop2,
    //                             kD10Loop3, kFinishApproach, kEndPose});

    if (!m_autonChooser.Suspend([=] { return m_drivetrain.AtGoal(); })) {
        return;
    }
}
}  // namespace frc3512
