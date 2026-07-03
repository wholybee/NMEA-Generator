// Nmea.h - NMEA 0183 sentence construction for ownship data.
#pragma once

#include <string>
#include <vector>
#include <ctime>

namespace nmea {

// Snapshot of ownship state used to build a batch of NMEA 0183 sentences.
struct OwnshipState {
    double latitude = 0.0;        // decimal degrees, + = N
    double longitude = 0.0;       // decimal degrees, + = E
    double cog = 0.0;             // course over ground, degrees true
    double sog = 0.0;             // speed over ground, knots
    double heading = 0.0;         // degrees true
    double speedThroughWater = 0.0; // knots
    double appWindAngle = 0.0;    // relative to bow, 0..360 degrees
    double appWindSpeed = 0.0;    // knots
    double trueWindDir = 0.0;     // degrees true
    double trueWindSpeed = 0.0;   // knots
};

// Compute the NMEA checksum (XOR of bytes between the leading '!'/'$' and '*').
unsigned char Checksum(const std::string& body);

// Wrap a sentence body (without leading talker char or trailing checksum) into
// a full sentence: "$" + body + "*" + HH + CRLF.
std::string Frame(char start, const std::string& body);

// Build the full set of ownship sentences (GGA, RMC, VTG, GLL, VHW, MWV x2,
// MWD) for the given state and UTC time.
std::vector<std::string> BuildOwnshipSentences(const OwnshipState& s, const std::tm& utc);

// ---- Incoming sentence parsing -------------------------------------------

// The 3-character sentence formatter (e.g. "APB", "RMB", "GGA") taken from the
// address field, upper-cased. Empty if the line is not a valid sentence.
std::string SentenceFormatter(const std::string& line);

// Verify the "*hh" checksum if present. Returns true when the checksum matches
// or when no checksum is present.
bool VerifyChecksum(const std::string& line);

// Split a sentence into comma-separated fields. Field 0 includes the leading
// "$"/"!" + address; any trailing "*hh" and CR/LF are stripped first.
std::vector<std::string> SplitFields(const std::string& line);

// Parse a NMEA lat/lon field pair (ddmm.mmmm + hemisphere) to decimal degrees.
// Returns false if the value field is empty/invalid.
bool ParseLatLon(const std::string& value, const std::string& hemi, double& out);

} // namespace nmea
