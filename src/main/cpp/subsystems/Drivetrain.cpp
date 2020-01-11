// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

using namespace frc3512;

Drivetrain::Drivetrain() : PublishNode("Drivetrain") {}

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {}

void Drivetrain::SetLeftManual(double value) {}

void Drivetrain::SetRightManual(double value) {}

double Drivetrain::GetAngle() const { return 0; }

double Drivetrain::GetAngularRate() const { return 0; }

void Drivetrain::ResetGyro() {}

void Drivetrain::CalibrateGyro() {}

double Drivetrain::GetLeftDisplacement() const { return 0; }

double Drivetrain::GetRightDisplacement() const { return 0; }

double Drivetrain::GetLeftRate() const { return 0; }

double Drivetrain::GetRightRate() const { return 0; }

void Drivetrain::ResetEncoders() {}

void Drivetrain::EnableController() {}

void Drivetrain::DisableController() {}

bool Drivetrain::IsControllerEnabled() const { return true; }

void Drivetrain::Reset() {}

void Drivetrain::Iterate() {}
