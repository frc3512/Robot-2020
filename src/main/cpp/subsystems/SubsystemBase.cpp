// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/SubsystemBase.hpp"

using namespace frc3512;

void SubsystemBase::EnablePeriodic() { m_notifier.StartPeriodic(20_ms); }

void SubsystemBase::DisablePeriodic() { m_notifier.Stop(); }

void SubsystemBase::SubsystemPeriodic() {}
