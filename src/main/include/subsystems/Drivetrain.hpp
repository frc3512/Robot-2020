// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/SpeedControllerGroup.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * Provides an interface for this year's drive train.
 */
class Drivetrain : public SubsystemBase, public PublishNode {
public:
    Drivetrain();
    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

    /**
     * Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

    /**
     * Directly set wheel speeds (see GearBox::SetManual(double)).
     *
     * @param value speeds [0..1]
     */
    void SetLeftManual(double value);

    /**
     * Directly set wheel speeds (see GearBox::SetManual(double)).
     *
     * @param value speeds [0..1]
     */
    void SetRightManual(double value);
    /**
     * Returns gyro angle.
     *
     * @return angle in degrees
     */
    double GetAngle() const;

    /**
     * Returns gyro angular rate.
     *
     * @return angular rate in degrees per second
     */
    double GetAngularRate() const;

    /**
     * Resets gyro.
     */
    void ResetGyro();

    /**
     * Calibrates gyro.
     */
    void CalibrateGyro();

    /**
     * Returns left encoder displacement.
     *
     * @return left displacement
     */
    double GetLeftDisplacement() const;

    /**
     * Returns right encoder displacement.
     *
     * @return right displacement
     */
    double GetRightDisplacement() const;

    /**
     * Returns right encoder displacement.
     *
     * @return left rate
     */
    double GetLeftRate() const;

    /**
     * Returns right encoder displacement.
     *
     * @return right rate
     */
    double GetRightRate() const;

    /**
     * Resets encoders.
     */
    void ResetEncoders();

    /**
     * Enable controller.
     */
    void EnableController();

    /**
     * Disable controller.
     */
    void DisableController();

    /**
     * Returns if the controller is enabled.
     *
     * @return if the controller is enabled
     */
    bool IsControllerEnabled() const;

    void Reset();

    void Iterate();

private:
    rev::CANSparkMax m_leftSlave{0, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftMaster{1, rev::CANSparkMax::MotorType::kBrushless};
    frc::SpeedControllerGroup m_leftGrbx{m_leftMaster, m_leftSlave};

    rev::CANSparkMax m_rightSlave{0, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightMaster{1, rev::CANSparkMax::MotorType::kBrushless};
    frc::SpeedControllerGroup m_rightGrbx{m_rightMaster, m_rightSlave};
};

}  // namespace frc3512
