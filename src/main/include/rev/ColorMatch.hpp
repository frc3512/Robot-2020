// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#if defined(__FRC_ROBORIO__)
#include <rev/ColorMatch.h>

#else

namespace rev {
/**
 * Rev Color Match Shim
 */
class ColorMatch {
public:
    void AddColorMatch(const frc::Color& color) { m_colorMatch = color; }

    frc::Color MatchClosestColor(const frc::Color& colorToMatch,
                                 double& confidence) {
        m_colorToMatch = colorToMatch;
        m_confidence = confidence;
        return frc::Color::kBlack;
    }

private:
    frc::Color m_colorToMatch;
    frc::Color m_colorMatch;
    double m_confidence;
};
}  // namespace rev
#endif
