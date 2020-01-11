// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

using namespace frc3512;

Drivetrain::Drivetrain() : PublishNode("Drivetrain") {}

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {}

void Drivetrain::SetLeftManual(double value) {}

void Drivetrain::SetRightManual(double value) {}

double Drivetrain::GetAngle() const {}

double Drivetrain::GetAngularRate() const {}

void Drivetrain::ResetGyro() {}

void Drivetrain::CalibrateGyro() {}

double Drivetrain::GetLeftDisplacement() const {}

double Drivetrain::GetRightDisplacement() const {}

double Drivetrain::GetLeftRate() const {}

double Drivetrain::GetRightRate() const {}

void Drivetrain::ResetEncoders() {}

void Drivetrain::EnableController() {}

void Drivetrain::DisableController() {}

bool Drivetrain::IsControllerEnabled() const {}

void Drivetrain::Reset() {}

void Drivetrain::Iterate() {}
