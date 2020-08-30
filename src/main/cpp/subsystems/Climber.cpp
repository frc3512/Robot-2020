// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <cmath>

using namespace frc3512;
using namespace frc3512::Constants::Climber;

void Climber::SetTransverser(double speed) { m_transverser.Set(speed); }

void Climber::SetElevator(double speed) { m_elevator.Set(speed); }
