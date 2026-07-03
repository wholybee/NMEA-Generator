// ApInput.h - Validation and decoding of inbound autopilot sentences
// (APB / RMB / XTE). A real autopilot is intolerant of malformed steering data,
// so each sentence is checked for completeness and correctness; any problems are
// reported and the key navigation fields are decoded for display.
#pragma once

#include <string>
#include <vector>

namespace nmea {

struct ApInput {
    std::string formatter;            // "APB", "RMB", "XTE", or "" if not one
    std::vector<std::string> errors;  // human-readable problems found

    bool ok() const { return !formatter.empty() && errors.empty(); }

    // Decoded fields (availability depends on sentence type and content).
    bool hasXte = false;          double xteNm = 0.0;  char xteDir = 0;   // 'L'/'R'
    bool hasHeadingToSteer = false; double headingToSteer = 0.0; bool htsMagnetic = false;
    bool hasBearingToDest = false;  double bearingToDest = 0.0;  bool btdMagnetic = false;
    bool hasDest = false;         double destLat = 0.0; double destLon = 0.0;
    bool hasRange = false;        double rangeNm = 0.0;

    // Bearing the autopilot model should steer (heading-to-steer / bearing).
    bool hasSteerBearing = false; double steerBearing = 0.0;

    // One-line "Cross-track ... | Heading-to-steer ... | Dest ..." summary.
    std::string DecodedSummary() const;
    // Errors joined with "; ".
    std::string ErrorText() const;
};

// Returns true if 'line' is an APB/RMB/XTE sentence (then 'out' is populated
// with any errors and the decoded fields); false if it is some other sentence.
bool ParseAutopilotInput(const std::string& line, ApInput& out);

} // namespace nmea
