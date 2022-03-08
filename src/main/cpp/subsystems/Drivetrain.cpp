// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <algorithm>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/StateSpaceUtil.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/fmt/Eigen.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "CANSparkMaxUtil.hpp"

using namespace frc3512;

const Eigen::Matrix<double, 2, 2> Drivetrain::kGlobalR =
    frc::MakeCovMatrix(0.2, 0.2);

const frc::LinearSystem<2, 2, 2> Drivetrain::kPlant{
    DrivetrainController::GetPlant()};

Drivetrain::Drivetrain()
    : ControlledSubsystemBase(
          "Drivetrain",
          {ControllerLabel{"X", "m"}, ControllerLabel{"Y", "m"},
           ControllerLabel{"Heading", "rad"},
           ControllerLabel{"Left velocity", "m/s"},
           ControllerLabel{"Right velocity", "m/s"},
           ControllerLabel{"Left position", "m"},
           ControllerLabel{"Right position", "m"}},
          {ControllerLabel{"Left voltage", "V"},
           ControllerLabel{"Right voltage", "V"}},
          {ControllerLabel{"Heading", "rad"},
           ControllerLabel{"Left position", "m"},
           ControllerLabel{"Right position", "m"},
           ControllerLabel{"Longitudinal Acceleration", "m/s^2"},
           ControllerLabel{"Lateral Acceleration", "m/s^2"}}) {
    SetCANSparkMaxBusUsage(m_leftLeader, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_leftFollower, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightLeader, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightFollower, Usage::kMinimal);

    m_leftLeader.SetSmartCurrentLimit(60);
    m_leftFollower.SetSmartCurrentLimit(60);
    m_rightLeader.SetSmartCurrentLimit(60);
    m_rightFollower.SetSmartCurrentLimit(60);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_leftGrbx.SetInverted(true);

    m_leftEncoder.SetSamplesToAverage(10);
    m_rightEncoder.SetSamplesToAverage(10);

    m_leftEncoder.SetReverseDirection(false);
    m_rightEncoder.SetReverseDirection(true);
    m_leftEncoder.SetDistancePerPulse(DrivetrainController::kDpP);
    m_rightEncoder.SetDistancePerPulse(DrivetrainController::kDpP);

    // Reset the pose estimate to the field's bottom-left corner with the turret
    // facing in the target's general direction. This is relatively close to the
    // robot's testing configuration, so the turret won't hit the soft limits.
    Reset(frc::Pose2d{0_m, 0_m, units::radian_t{wpi::numbers::pi}});

    frc::SmartDashboard::PutData(&m_field);
}

frc::Pose2d Drivetrain::GetReferencePose() const {
    const auto& x = m_controller.GetReferences();
    return frc::Pose2d{
        units::meter_t{x(DrivetrainController::State::kX)},
        units::meter_t{x(DrivetrainController::State::kY)},
        units::radian_t{x(DrivetrainController::State::kHeading)}};
}

frc::Pose2d Drivetrain::GetPose() const {
    const auto& x = m_observer.Xhat();
    return frc::Pose2d{
        units::meter_t{x(DrivetrainController::State::kX)},
        units::meter_t{x(DrivetrainController::State::kY)},
        units::radian_t{x(DrivetrainController::State::kHeading)}};
}

units::meter_t Drivetrain::GetLeftUltrasonicDistance() const {
    return m_leftUltrasonicDistance;
}

units::meter_t Drivetrain::GetRightUltrasonicDistance() const {
    return m_rightUltrasonicDistance;
}

units::radian_t Drivetrain::GetAngle() const {
    return units::degree_t{m_imu.GetAngle()} + m_headingOffset;
}

units::meter_t Drivetrain::GetLeftPosition() const {
    return units::meter_t{m_leftEncoder.GetDistance()};
}

units::meter_t Drivetrain::GetRightPosition() const {
    return units::meter_t{m_rightEncoder.GetDistance()};
}

units::meters_per_second_t Drivetrain::GetLeftVelocity() const {
    return units::meters_per_second_t{m_leftEncoder.GetRate()};
}

units::meters_per_second_t Drivetrain::GetRightVelocity() const {
    return units::meters_per_second_t{m_rightEncoder.GetRate()};
}

units::meters_per_second_squared_t Drivetrain::GetAccelerationX() const {
    return -m_imu.GetAccelX();
}

units::meters_per_second_squared_t Drivetrain::GetAccelerationY() const {
    return -m_imu.GetAccelY();
}

void Drivetrain::Reset(const frc::Pose2d& initialPose) {
    using State = frc::sim::DifferentialDrivetrainSim::State;

    m_observer.Reset();
    m_controller.Reset(initialPose);
    m_u = Eigen::Vector<double, 2>::Zero();
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
    m_imu.Reset();
    m_headingOffset = initialPose.Rotation().Radians();

    Eigen::Vector<double, 7> xHat;
    xHat(State::kX) = initialPose.X().value();
    xHat(State::kY) = initialPose.Y().value();
    xHat(State::kHeading) = initialPose.Rotation().Radians().value();
    xHat.block<4, 1>(3, 0).setZero();
    m_observer.SetXhat(xHat);

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_drivetrainSim.SetState(xHat);
        m_field.SetRobotPose(initialPose);
    }
}

void Drivetrain::CorrectWithGlobalOutputs(units::meter_t x, units::meter_t y,
                                          units::second_t timestamp) {
    Eigen::Vector<double, 2> globalY{x.value(), y.value()};
    m_latencyComp.ApplyPastGlobalMeasurement<2>(
        &m_observer, Constants::kControllerPeriod, globalY,
        [&](const Eigen::Vector<double, 2>& u,
            const Eigen::Vector<double, 2>& y) {
            m_observer.Correct<2>(
                u, y, &DrivetrainController::GlobalMeasurementModel, kGlobalR);
        },
        timestamp);
}

void Drivetrain::ControllerPeriodic() {
    using Input = DrivetrainController::Input;

    UpdateDt();

    m_observer.Predict(m_u, GetDt());

    Eigen::Vector<double, 5> y{
        frc::AngleModulus(GetAngle()).value(), GetLeftPosition().value(),
        GetRightPosition().value(), GetAccelerationX().value(),
        GetAccelerationY().value()};
    m_latencyComp.AddObserverState(m_observer, m_controller.GetInputs(), y,
                                   frc::Timer::GetFPGATimestamp());
    m_observer.Correct(m_controller.GetInputs(), y);

    while (visionQueue.size() > 0) {
        auto measurement = visionQueue.pop_front();

        // If pose measurement is too far away from the state estimate, discard
        // it and increment the fault counter
        if (GetPose().Translation().Distance(
                measurement.drivetrainInGlobal.Translation()) < 1_m) {
            CorrectWithGlobalOutputs(measurement.drivetrainInGlobal.X(),
                                     measurement.drivetrainInGlobal.Y(),
                                     measurement.timestamp);
        } else {
            m_poseMeasurementFaultCounter++;
        }
    }

    if (m_controller.HaveTrajectory()) {
        m_u = m_controller.Calculate(m_observer.Xhat());

        if (!AtGoal()) {
            m_leftGrbx.SetVoltage(units::volt_t{m_u(Input::kLeftVoltage)});
            m_rightGrbx.SetVoltage(units::volt_t{m_u(Input::kRightVoltage)});
        } else {
            m_leftGrbx.SetVoltage(0_V);
            m_rightGrbx.SetVoltage(0_V);
        }
    } else {
        // Update previous u stored in the controller. We don't care what the
        // return value is.
        static_cast<void>(m_controller.Calculate(m_observer.Xhat()));

        // Run observer predict with inputs from teleop
        m_u = Eigen::Vector<double, 2>{
            std::clamp(m_leftGrbx.Get(), -1.0, 1.0) *
                frc::RobotController::GetInputVoltage(),
            std::clamp(m_rightGrbx.Get(), -1.0, 1.0) *
                frc::RobotController::GetInputVoltage()};
    }

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);

    if constexpr (frc::RobotBase::IsSimulation()) {
        auto batteryVoltage = frc::RobotController::GetInputVoltage();
        m_drivetrainSim.SetInputs(
            units::volt_t{std::clamp(m_leftGrbx.Get(), -1.0, 1.0) *
                          batteryVoltage},
            units::volt_t{std::clamp(m_rightGrbx.Get(), -1.0, 1.0) *
                          batteryVoltage});

        m_drivetrainSim.Update(GetDt());

        m_leftEncoderSim.SetDistance(m_drivetrainSim.GetLeftPosition().value());
        m_leftEncoderSim.SetRate(m_drivetrainSim.GetLeftVelocity().value());
        m_rightEncoderSim.SetDistance(
            m_drivetrainSim.GetRightPosition().value());
        m_rightEncoderSim.SetRate(m_drivetrainSim.GetLeftVelocity().value());
        m_imuSim.SetGyroAngleZ(units::degree_t{
            m_drivetrainSim.GetHeading().Radians() - m_headingOffset});

        const auto& plant = DrivetrainController::GetPlant();
        Eigen::Vector<double, 2> x{m_drivetrainSim.GetLeftVelocity().value(),
                                   m_drivetrainSim.GetRightVelocity().value()};
        Eigen::Vector<double, 2> u{
            std::clamp(m_leftGrbx.Get(), -1.0, 1.0) * batteryVoltage,
            std::clamp(m_rightGrbx.Get(), -1.0, 1.0) * batteryVoltage};
        Eigen::Vector<double, 2> xdot = plant.A() * x + plant.B() * u;
        m_imuSim.SetAccelX(
            -units::meters_per_second_squared_t{(xdot(0) + xdot(1)) / 2.0});

        units::meters_per_second_t leftVelocity{x(0)};
        units::meters_per_second_t rightVelocity{x(1)};
        m_imuSim.SetAccelY(
            -((rightVelocity * rightVelocity) - (leftVelocity * leftVelocity)) /
            (2.0 * DrivetrainController::kWidth));

        m_field.SetRobotPose(m_drivetrainSim.GetPose());
    }
}

void Drivetrain::RobotPeriodic() {
    m_leftUltrasonicDistance = m_leftDistanceFilter.Calculate(
        units::volt_t{m_leftUltrasonic.GetVoltage()} * 1_m / 1_V);
    m_rightUltrasonicDistance = m_rightDistanceFilter.Calculate(
        units::volt_t{m_rightUltrasonic.GetVoltage()} * 1_m / 1_V);
    m_poseMeasurementFaultEntry.SetDouble(m_poseMeasurementFaultCounter);

    if (frc::DriverStation::IsDisabled() ||
        !frc::DriverStation::IsFMSAttached()) {
        m_leftUltrasonicOutputEntry.SetDouble(m_leftUltrasonicDistance.value());
        m_rightUltrasonicOutputEntry.SetDouble(
            m_rightUltrasonicDistance.value());
    }

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_leftUltrasonicDistance = m_leftDistanceFilter.Calculate(
            units::volt_t{m_leftUltrasonicSim.GetVoltage()} * 1_m / 1_V);
        m_rightUltrasonicDistance = m_rightDistanceFilter.Calculate(
            units::volt_t{m_rightUltrasonicSim.GetVoltage()} * 1_m / 1_V);
    }
}

void Drivetrain::AddTrajectory(const frc::Pose2d& start,
                               const std::vector<frc::Translation2d>& interior,
                               const frc::Pose2d& end) {
    m_controller.AddTrajectory(start, interior, end);
}

void Drivetrain::AddTrajectory(const frc::Pose2d& start,
                               const std::vector<frc::Translation2d>& interior,
                               const frc::Pose2d& end,
                               const frc::TrajectoryConfig& config) {
    m_controller.AddTrajectory(start, interior, end, config);
}

void Drivetrain::AddTrajectory(const std::vector<frc::Pose2d>& waypoints) {
    m_controller.AddTrajectory(waypoints);
}

void Drivetrain::AddTrajectory(const std::vector<frc::Pose2d>& waypoints,
                               const frc::TrajectoryConfig& config) {
    m_controller.AddTrajectory(waypoints, config);
}

frc::TrajectoryConfig Drivetrain::MakeTrajectoryConfig() {
    return DrivetrainController::MakeTrajectoryConfig();
}

frc::TrajectoryConfig Drivetrain::MakeTrajectoryConfig(
    units::meters_per_second_t startVelocity,
    units::meters_per_second_t endVelocity) {
    return DrivetrainController::MakeTrajectoryConfig(startVelocity,
                                                      endVelocity);
}

bool Drivetrain::AtGoal() const { return m_controller.AtGoal(); }

const Eigen::Vector<double, 7>& Drivetrain::GetStates() const {
    return m_observer.Xhat();
}

const Eigen::Vector<double, 2>& Drivetrain::GetInputs() const {
    return m_controller.GetInputs();
}

units::ampere_t Drivetrain::GetCurrentDraw() const {
    return m_drivetrainSim.GetCurrentDraw();
}

int Drivetrain::GetPoseMeasurementFaultCounter() {
    return m_poseMeasurementFaultCounter;
}

frc::Pose2d Drivetrain::GetSimPose() const { return m_drivetrainSim.GetPose(); }

void Drivetrain::DisabledInit() {
    SetCoastMode();
    Disable();
}

void Drivetrain::AutonomousInit() {
    SetBrakeMode();
    Enable();
}

void Drivetrain::TeleopInit() {
    SetBrakeMode();

    // If the robot was disabled while still following a trajectory in
    // autonomous, it will continue to do so in teleop. This aborts any
    // trajectories so teleop driving can occur.
    m_controller.AbortTrajectories();

    Enable();
}

void Drivetrain::TestInit() {
    SetBrakeMode();

    // If the robot was disabled while still following a trajectory in
    // autonomous, it will continue to do so in teleop. This aborts any
    // trajectories so teleop driving can occur.
    m_controller.AbortTrajectories();

    Enable();
}

void Drivetrain::TeleopPeriodic() {
    using Input = DrivetrainController::Input;

    static frc::Joystick driveStick1{HWConfig::kDriveStick1Port};
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};

    double y =
        frc::ApplyDeadband(-driveStick1.GetY(), Constants::kJoystickDeadband);
    double x =
        frc::ApplyDeadband(driveStick2.GetX(), Constants::kJoystickDeadband);

    if (driveStick1.GetRawButton(1)) {
        y *= 0.5;
        x *= 0.5;
    }
    auto [left, right] = frc::DifferentialDrive::CurvatureDriveIK(
        y, x, driveStick2.GetRawButton(2));

    // Implicit model following
    // TODO: Velocities need filtering
    Eigen::Vector<double, 2> u =
        m_imf.Calculate(Eigen::Vector<double, 2>{GetLeftVelocity().value(),
                                                 GetRightVelocity().value()},
                        Eigen::Vector<double, 2>{left * 12.0, right * 12.0});

    m_leftGrbx.SetVoltage(units::volt_t{u(Input::kLeftVoltage)});
    m_rightGrbx.SetVoltage(units::volt_t{u(Input::kRightVoltage)});
}

void Drivetrain::TestPeriodic() {
    using Input = DrivetrainController::Input;

    static frc::Joystick driveStick1{HWConfig::kDriveStick1Port};
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};

    double y =
        frc::ApplyDeadband(-driveStick1.GetY(), Constants::kJoystickDeadband);
    double x =
        frc::ApplyDeadband(driveStick2.GetX(), Constants::kJoystickDeadband);

    if (driveStick1.GetRawButton(1)) {
        y *= 0.5;
        x *= 0.5;
    }
    auto [left, right] = frc::DifferentialDrive::CurvatureDriveIK(
        y, x, driveStick2.GetRawButton(2));

    // Implicit model following
    // TODO: Velocities need filtering
    Eigen::Vector<double, 2> u =
        m_imf.Calculate(Eigen::Vector<double, 2>{GetLeftVelocity().value(),
                                                 GetRightVelocity().value()},
                        Eigen::Vector<double, 2>{left * 12.0, right * 12.0});

    m_leftGrbx.SetVoltage(units::volt_t{u(Input::kLeftVoltage)});
    m_rightGrbx.SetVoltage(units::volt_t{u(Input::kRightVoltage)});
}

void Drivetrain::SetBrakeMode() {
    m_leftLeader.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_leftFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightLeader.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Drivetrain::SetCoastMode() {
    m_leftLeader.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightLeader.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}
