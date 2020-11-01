// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "logging/LiveGrapherControllerLogger.hpp"

namespace frc3512 {
LiveGrapher& GetGrapher() {
    static LiveGrapher liveGrapher{3513};
    return liveGrapher;
}
}  // namespace frc3512
