// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

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
     * @return displacement
     */
    double GetLeftDisplacement() const;

    /**
     * Returns right encoder displacement.
     *
     * @return displacement
     */
    double GetRightDisplacement() const;

    double GetLeftRate() const;

    double GetRightRate() const;

    void ResetEncoders();

    void EnableController();

    void DisableController();

    bool IsControllerEnabled() const;

    void Reset();

    void Iterate();

private:
};

}  // namespace frc3512
