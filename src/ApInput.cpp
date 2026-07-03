// ApInput.cpp - Validation and decoding of inbound autopilot sentences.
#include "ApInput.h"
#include "Nmea.h"

#include <cstdio>
#include <cstdlib>
#include <cmath>

namespace nmea {

namespace {

const std::string kEmpty;

const std::string& At(const std::vector<std::string>& f, size_t i) {
    return i < f.size() ? f[i] : kEmpty;
}

bool IsStatus(const std::string& s) { return s == "A" || s == "V"; }
bool IsLR(const std::string& s) { return s == "L" || s == "R"; }
bool IsMT(const std::string& s) { return s == "M" || s == "T"; }

// Parse a field that must be a complete number.
bool Num(const std::string& s, double& out) {
    if (s.empty()) return false;
    char* end = nullptr;
    out = std::strtod(s.c_str(), &end);
    return end && *end == '\0';
}

// Describe a checksum mismatch with the received vs expected value.
std::string ChecksumDetail(const std::string& line) {
    size_t star = line.find('*');
    unsigned char cs = 0;
    for (size_t i = 1; i < star; ++i) cs ^= static_cast<unsigned char>(line[i]);
    char expected[4];
    std::snprintf(expected, sizeof(expected), "%02X", cs);
    std::string got = (star + 2 < line.size()) ? line.substr(star + 1, 2) : "??";
    return "checksum mismatch (received *" + got + ", computed *" + expected + ")";
}

// Validate a NMEA latitude/longitude pair, returning a problem description in
// 'why' on failure. 'isLat' selects 90/180 range and N-S vs E-W hemispheres.
bool ValidateLatLon(const std::string& value, const std::string& hemi,
                    bool isLat, double& out, std::string& why) {
    if (value.empty()) { why = "value missing"; return false; }
    double v;
    if (!Num(value, v)) { why = "not numeric: '" + value + "'"; return false; }
    int deg = static_cast<int>(v / 100.0);
    double min = v - deg * 100.0;
    if (min >= 60.0) { why = "minutes field >= 60"; return false; }
    double dec = deg + min / 60.0;
    if (dec > (isLat ? 90.0 : 180.0)) { why = "magnitude out of range"; return false; }
    const char* valid = isLat ? "N/S" : "E/W";
    if (hemi.empty()) { why = std::string("hemisphere missing (expected ") + valid + ")"; return false; }
    if (isLat ? (hemi != "N" && hemi != "S") : (hemi != "E" && hemi != "W")) {
        why = std::string("invalid hemisphere '") + hemi + "' (expected " + valid + ")";
        return false;
    }
    out = (hemi == "S" || hemi == "W") ? -dec : dec;
    return true;
}

std::string FmtLat(double lat) {
    char h = lat >= 0 ? 'N' : 'S';
    double a = std::fabs(lat);
    int d = static_cast<int>(a);
    char b[40];
    std::snprintf(b, sizeof(b), "%02d°%05.2f'%c", d, (a - d) * 60.0, h);
    return b;
}
std::string FmtLon(double lon) {
    char h = lon >= 0 ? 'E' : 'W';
    double a = std::fabs(lon);
    int d = static_cast<int>(a);
    char b[40];
    std::snprintf(b, sizeof(b), "%03d°%05.2f'%c", d, (a - d) * 60.0, h);
    return b;
}

// --- per-sentence validators ----------------------------------------------

void ParseXte(const std::vector<std::string>& f, ApInput& o) {
    auto err = [&](const std::string& s) { o.errors.push_back(s); };
    if (f.size() < 6)
        err("incomplete XTE: expected at least 6 fields, got " + std::to_string(f.size()));

    if (!IsStatus(At(f, 1))) err("field 1 (status) must be A or V");
    else if (At(f, 1) == "V") err("status V: cross-track data reported not valid");
    if (!At(f, 2).empty() && !IsStatus(At(f, 2))) err("field 2 (status) must be A or V");

    double x;
    if (At(f, 3).empty()) err("field 3 (cross-track error) missing");
    else if (!Num(At(f, 3), x)) err("field 3 (cross-track error) not numeric: '" + At(f, 3) + "'");
    else { o.hasXte = true; o.xteNm = x; }

    if (!IsLR(At(f, 4))) err("field 4 (direction to steer) must be L or R");
    else o.xteDir = At(f, 4)[0];

    if (At(f, 5) != "N") err("field 5 (units) must be N, got '" + At(f, 5) + "'");
}

void ParseApb(const std::vector<std::string>& f, ApInput& o) {
    auto err = [&](const std::string& s) { o.errors.push_back(s); };
    if (f.size() < 15)
        err("incomplete APB: expected 15 fields, got " + std::to_string(f.size()));

    if (!IsStatus(At(f, 1))) err("field 1 (status) must be A or V");
    else if (At(f, 1) == "V") err("status V: data reported not valid");
    if (!IsStatus(At(f, 2))) err("field 2 (status) must be A or V");
    else if (At(f, 2) == "V") err("status V: data reported not valid");

    double x;
    if (At(f, 3).empty()) err("field 3 (cross-track error) missing");
    else if (!Num(At(f, 3), x)) err("field 3 (cross-track error) not numeric: '" + At(f, 3) + "'");
    else { o.hasXte = true; o.xteNm = x; }

    if (!IsLR(At(f, 4))) err("field 4 (direction to steer) must be L or R");
    else o.xteDir = At(f, 4)[0];

    if (At(f, 5) != "N") err("field 5 (units) must be N, got '" + At(f, 5) + "'");
    if (!IsStatus(At(f, 6))) err("field 6 (arrival circle status) must be A or V");
    if (!IsStatus(At(f, 7))) err("field 7 (perpendicular status) must be A or V");

    double b;
    if (At(f, 8).empty() || !Num(At(f, 8), b)) err("field 8 (bearing origin to destination) invalid");
    if (!IsMT(At(f, 9))) err("field 9 (bearing reference) must be M or T");

    if (At(f, 11).empty()) err("field 11 (bearing to destination) missing");
    else if (!Num(At(f, 11), b)) err("field 11 (bearing to destination) not numeric: '" + At(f, 11) + "'");
    else { o.hasBearingToDest = true; o.bearingToDest = b; o.btdMagnetic = (At(f, 12) == "M"); }
    if (!IsMT(At(f, 12))) err("field 12 (bearing reference) must be M or T");

    if (At(f, 13).empty()) err("field 13 (heading to steer) missing");
    else if (!Num(At(f, 13), b)) err("field 13 (heading to steer) not numeric: '" + At(f, 13) + "'");
    else { o.hasHeadingToSteer = true; o.headingToSteer = b; o.htsMagnetic = (At(f, 14) == "M"); }
    if (!IsMT(At(f, 14))) err("field 14 (heading reference) must be M or T");

    if (o.hasHeadingToSteer) { o.hasSteerBearing = true; o.steerBearing = o.headingToSteer; }
    else if (o.hasBearingToDest) { o.hasSteerBearing = true; o.steerBearing = o.bearingToDest; }
}

void ParseRmb(const std::vector<std::string>& f, ApInput& o) {
    auto err = [&](const std::string& s) { o.errors.push_back(s); };
    if (f.size() < 14)
        err("incomplete RMB: expected 14 fields, got " + std::to_string(f.size()));

    if (!IsStatus(At(f, 1))) err("field 1 (status) must be A or V");
    else if (At(f, 1) == "V") err("status V: navigation data reported not valid");

    double x;
    if (At(f, 2).empty()) err("field 2 (cross-track error) missing");
    else if (!Num(At(f, 2), x)) err("field 2 (cross-track error) not numeric: '" + At(f, 2) + "'");
    else { o.hasXte = true; o.xteNm = x; }

    if (!IsLR(At(f, 3))) err("field 3 (direction to steer) must be L or R");
    else o.xteDir = At(f, 3)[0];

    double dlat, dlon; std::string why;
    bool latOk = ValidateLatLon(At(f, 6), At(f, 7), true, dlat, why);
    if (!latOk) err("field 6/7 (destination latitude) " + why);
    bool lonOk = ValidateLatLon(At(f, 8), At(f, 9), false, dlon, why);
    if (!lonOk) err("field 8/9 (destination longitude) " + why);
    if (latOk && lonOk) { o.hasDest = true; o.destLat = dlat; o.destLon = dlon; }

    double r;
    if (At(f, 10).empty()) err("field 10 (range to destination) missing");
    else if (!Num(At(f, 10), r)) err("field 10 (range to destination) not numeric: '" + At(f, 10) + "'");
    else if (r < 0.0) err("field 10 (range to destination) negative");
    else { o.hasRange = true; o.rangeNm = r; }

    double b;
    if (At(f, 11).empty()) err("field 11 (bearing to destination) missing");
    else if (!Num(At(f, 11), b)) err("field 11 (bearing to destination) not numeric: '" + At(f, 11) + "'");
    else if (b < 0.0 || b > 360.0) err("field 11 (bearing to destination) out of range 0-360");
    else { o.hasBearingToDest = true; o.bearingToDest = b; o.btdMagnetic = false; }

    if (!At(f, 12).empty() && !Num(At(f, 12), b))
        err("field 12 (closing velocity) not numeric: '" + At(f, 12) + "'");

    if (!IsStatus(At(f, 13))) err("field 13 (arrival status) must be A or V");

    if (o.hasBearingToDest) { o.hasSteerBearing = true; o.steerBearing = o.bearingToDest; }
}

} // namespace

std::string ApInput::ErrorText() const {
    std::string s;
    for (size_t i = 0; i < errors.size(); ++i) { if (i) s += "; "; s += errors[i]; }
    return s;
}

std::string ApInput::DecodedSummary() const {
    std::vector<std::string> parts;
    char b[80];
    if (hasXte) {
        const char* d = xteDir == 'L' ? " (steer L)" : xteDir == 'R' ? " (steer R)" : "";
        std::snprintf(b, sizeof(b), "Cross-track %.2f NM%s", xteNm, d);
        parts.push_back(b);
    }
    if (hasHeadingToSteer) {
        std::snprintf(b, sizeof(b), "Heading-to-steer %05.1f°%c",
                      headingToSteer, htsMagnetic ? 'M' : 'T');
        parts.push_back(b);
    }
    if (hasDest) parts.push_back("Dest " + FmtLat(destLat) + " " + FmtLon(destLon));
    if (hasBearingToDest) {
        std::snprintf(b, sizeof(b), "Bearing-to-dest %05.1f°%c",
                      bearingToDest, btdMagnetic ? 'M' : 'T');
        parts.push_back(b);
    }
    if (hasRange) {
        std::snprintf(b, sizeof(b), "Range-to-dest %.2f NM", rangeNm);
        parts.push_back(b);
    }
    std::string s;
    for (size_t i = 0; i < parts.size(); ++i) { if (i) s += " | "; s += parts[i]; }
    return s.empty() ? "(no decodable fields)" : s;
}

bool ParseAutopilotInput(const std::string& line, ApInput& out) {
    out = ApInput{};
    const std::string fmt = SentenceFormatter(line);
    if (fmt != "APB" && fmt != "RMB" && fmt != "XTE") return false;
    out.formatter = fmt;

    if (line.empty() || line[0] != '$')
        out.errors.push_back("sentence does not start with '$'");
    if (line.find('*') == std::string::npos)
        out.errors.push_back("missing checksum");
    else if (!VerifyChecksum(line))
        out.errors.push_back(ChecksumDetail(line));

    const std::vector<std::string> f = SplitFields(line);
    if (fmt == "XTE") ParseXte(f, out);
    else if (fmt == "APB") ParseApb(f, out);
    else ParseRmb(f, out);

    return true;
}

} // namespace nmea
