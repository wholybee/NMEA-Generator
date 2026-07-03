// N2k.cpp - NMEA 2000 PGN payload encoding over Actisense N2K ASCII lines.
#include "N2k.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <sstream>
#include <ctime>

namespace nmea {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kMetersPerNm = 1852.0;

void U8(std::vector<uint8_t>& d, uint8_t v) { d.push_back(v); }
void U16(std::vector<uint8_t>& d, uint16_t v) {
    d.push_back(static_cast<uint8_t>(v & 0xff));
    d.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
}
void I16(std::vector<uint8_t>& d, int16_t v) { U16(d, static_cast<uint16_t>(v)); }
void U32(std::vector<uint8_t>& d, uint32_t v) {
    d.push_back(static_cast<uint8_t>(v & 0xff));
    d.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
    d.push_back(static_cast<uint8_t>((v >> 16) & 0xff));
    d.push_back(static_cast<uint8_t>((v >> 24) & 0xff));
}
void I32(std::vector<uint8_t>& d, int32_t v) { U32(d, static_cast<uint32_t>(v)); }
void U24(std::vector<uint8_t>& d, uint32_t v) {
    d.push_back(static_cast<uint8_t>(v & 0xff));
    d.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
    d.push_back(static_cast<uint8_t>((v >> 16) & 0xff));
}

bool Need(const std::vector<uint8_t>& d, size_t n, ApInput& out, const char* name) {
    if (d.size() >= n) return true;
    out.errors.push_back(std::string("incomplete ") + name + ": expected at least " +
                         std::to_string(n) + " payload bytes, got " + std::to_string(d.size()));
    return false;
}

uint16_t RdU16(const std::vector<uint8_t>& d, size_t o) {
    return static_cast<uint16_t>(d[o] | (d[o + 1] << 8));
}
int32_t RdI32(const std::vector<uint8_t>& d, size_t o) {
    uint32_t v = static_cast<uint32_t>(d[o]) |
                 (static_cast<uint32_t>(d[o + 1]) << 8) |
                 (static_cast<uint32_t>(d[o + 2]) << 16) |
                 (static_cast<uint32_t>(d[o + 3]) << 24);
    return static_cast<int32_t>(v);
}
uint32_t RdU32(const std::vector<uint8_t>& d, size_t o) {
    return static_cast<uint32_t>(d[o]) |
           (static_cast<uint32_t>(d[o + 1]) << 8) |
           (static_cast<uint32_t>(d[o + 2]) << 16) |
           (static_cast<uint32_t>(d[o + 3]) << 24);
}

uint16_t Angle(double deg) {
    while (deg < 0.0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    long v = std::lround(deg * kDegToRad * 10000.0);
    if (v < 0) v = 0;
    if (v > 65534) v = 65534;
    return static_cast<uint16_t>(v);
}

double AngleDeg(uint16_t raw) {
    if (raw == 0xffff) return 0.0;
    return raw / 10000.0 * kRadToDeg;
}

uint16_t Speed(double knots) {
    long v = std::lround(knots * 0.514444 * 100.0); // 0.01 m/s
    if (v < 0) v = 0;
    if (v > 65534) v = 65534;
    return static_cast<uint16_t>(v);
}

int32_t LatLon(double deg) {
    double scaled = deg * 10000000.0;
    if (scaled > 2147483647.0) scaled = 2147483647.0;
    if (scaled < -2147483648.0) scaled = -2147483648.0;
    return static_cast<int32_t>(std::llround(scaled));
}

double LatLonDeg(int32_t raw) { return raw / 10000000.0; }

void FixedString(std::vector<uint8_t>& d, const std::string& s, size_t width) {
    for (size_t i = 0; i < width; ++i)
        d.push_back(i < s.size() ? static_cast<uint8_t>(s[i]) : static_cast<uint8_t>(' '));
}

uint16_t Decimeters(double metres) {
    long v = std::lround(metres * 10.0);
    if (v < 0) v = 0;
    if (v > 65532) v = 65532;
    return static_cast<uint16_t>(v);
}

int16_t RotRaw(double degPerMinute) {
    const double radPerSecond = degPerMinute * kDegToRad / 60.0;
    long v = std::lround(radPerSecond / 0.00003125);
    if (v > 32764) v = 32764;
    if (v < -32767) v = -32767;
    return static_cast<int16_t>(v);
}

uint8_t AisMsgRepeatByte(uint8_t messageId, uint8_t repeat = 0) {
    return static_cast<uint8_t>((messageId & 0x3f) | ((repeat & 0x03) << 6));
}

uint8_t AisTimestampByte(int timestamp, bool posAccuracy = false, bool raim = false) {
    if (timestamp < 0 || timestamp > 59) timestamp = 60; // unavailable / invalid
    return static_cast<uint8_t>((posAccuracy ? 1 : 0) |
                                (raim ? 2 : 0) |
                                ((timestamp & 0x3f) << 2));
}

bool IsHex(const std::string& s) {
    return std::all_of(s.begin(), s.end(), [](unsigned char c) {
        return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
    });
}

bool IsDecDigits(const std::string& s) {
    return std::all_of(s.begin(), s.end(), [](unsigned char c) {
        return c >= '0' && c <= '9';
    });
}

bool IsActisenseTime(const std::string& s) {
    if (s.size() == 6) return IsDecDigits(s);
    if (s.size() == 10 && s[6] == '.')
        return IsDecDigits(s.substr(0, 6)) && IsDecDigits(s.substr(7, 3));
    return false;
}

std::string CurrentActisenseTime() {
    std::time_t t = std::time(nullptr);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &t);
#else
    gmtime_r(&t, &utc);
#endif
    char b[16];
    std::snprintf(b, sizeof(b), "%02d%02d%02d.000", utc.tm_hour, utc.tm_min, utc.tm_sec);
    return b;
}

bool ParseHexByte(const std::string& s, uint8_t& out) {
    if (s.size() != 2 || !IsHex(s)) return false;
    char* end = nullptr;
    long v = std::strtol(s.c_str(), &end, 16);
    if (!end || *end != '\0' || v < 0 || v > 255) return false;
    out = static_cast<uint8_t>(v);
    return true;
}

std::string HexBytes(const std::vector<uint8_t>& data) {
    static const char* h = "0123456789ABCDEF";
    std::string s;
    s.reserve(data.size() * 2);
    for (uint8_t b : data) {
        s.push_back(h[b >> 4]);
        s.push_back(h[b & 0x0f]);
    }
    return s;
}

uint8_t DefaultPriority(uint32_t pgn) {
    switch (pgn) {
        case kPgnPositionRapid:
        case kPgnCogSogRapid:
        case kPgnHeading:
        case kPgnWaterSpeed:
        case kPgnWindData:
            return 2;
        case kPgnAisClassAPosition:
        case kPgnAisClassBPosition:
        case kPgnAisClassAStatic:
        case kPgnAisSarAircraft:
        case kPgnAisSafetyBroadcast:
        case kPgnAisClassBStaticA:
        case kPgnAisClassBStaticB:
            return 4;
        default:
            return 3;
    }
}

std::vector<uint8_t> OwnshipSystemTime(const std::tm& utc) {
    std::vector<uint8_t> d;
    U8(d, 0);
    U16(d, static_cast<uint16_t>(utc.tm_yday)); // day-of-year for this simulator transport
    uint32_t ms = static_cast<uint32_t>(((utc.tm_hour * 60 + utc.tm_min) * 60 + utc.tm_sec) * 1000);
    U32(d, ms);
    return d;
}

std::vector<uint8_t> OwnshipGnss(const OwnshipState& s) {
    std::vector<uint8_t> d;
    U8(d, 0);
    I32(d, LatLon(s.latitude));
    I32(d, LatLon(s.longitude));
    I32(d, 0);       // altitude, 0.01 m
    U8(d, 3);        // GNSS fix
    U8(d, 8);        // satellites
    U16(d, 90);      // HDOP, 0.01
    return d;
}

void ParseCrossTrack(const N2kMessage& m, ApInput& out) {
    out.formatter = "N2K 129283";
    if (!Need(m.data, 6, out, "PGN 129283")) return;
    if (m.data[1] != 0) out.errors.push_back("field 1 (XTE mode) must be 0 for autonomous");
    int32_t cm = RdI32(m.data, 2);
    out.hasXte = true;
    out.xteNm = std::fabs(cm / 100.0) / kMetersPerNm;
    out.xteDir = cm < 0 ? 'L' : 'R';
}

void ParseTrackControl(const N2kMessage& m, ApInput& out) {
    out.formatter = "N2K 127237";
    if (!Need(m.data, 6, out, "PGN 127237")) return;
    uint16_t hts = RdU16(m.data, 1);
    uint16_t btd = RdU16(m.data, 3);
    if (hts == 0xffff) out.errors.push_back("heading-to-steer unavailable");
    else {
        out.hasHeadingToSteer = true;
        out.headingToSteer = AngleDeg(hts);
        out.hasSteerBearing = true;
        out.steerBearing = out.headingToSteer;
    }
    if (btd != 0xffff) {
        out.hasBearingToDest = true;
        out.bearingToDest = AngleDeg(btd);
    }
}

void ParseNavigation(const N2kMessage& m, ApInput& out) {
    out.formatter = "N2K 129284";
    if (!Need(m.data, 18, out, "PGN 129284")) return;
    uint32_t rangeCm = RdU32(m.data, 1);
    uint16_t btd = RdU16(m.data, 7);
    int32_t lat = RdI32(m.data, 9);
    int32_t lon = RdI32(m.data, 13);
    if (rangeCm == 0xffffffff) out.errors.push_back("range to destination unavailable");
    else { out.hasRange = true; out.rangeNm = (rangeCm / 100.0) / kMetersPerNm; }
    if (btd == 0xffff) out.errors.push_back("bearing to destination unavailable");
    else {
        out.hasBearingToDest = true;
        out.bearingToDest = AngleDeg(btd);
        out.hasSteerBearing = true;
        out.steerBearing = out.bearingToDest;
    }
    double dlat = LatLonDeg(lat);
    double dlon = LatLonDeg(lon);
    if (std::fabs(dlat) > 90.0) out.errors.push_back("destination latitude out of range");
    if (std::fabs(dlon) > 180.0) out.errors.push_back("destination longitude out of range");
    if (std::fabs(dlat) <= 90.0 && std::fabs(dlon) <= 180.0) {
        out.hasDest = true;
        out.destLat = dlat;
        out.destLon = dlon;
    }
}

} // namespace

std::string EncodeActisenseAscii(uint32_t pgn, const std::vector<uint8_t>& data,
                                 uint8_t source, uint8_t destination,
                                 uint8_t priority) {
    if (priority > 7) priority = DefaultPriority(pgn);
    char header[64];
    std::snprintf(header, sizeof(header), "A%s %02X%02X%X %05X ",
                  CurrentActisenseTime().c_str(),
                  source, destination, priority & 0x0f,
                  pgn & 0x3ffffu);
    return std::string(header) + HexBytes(data) + "\r\n";
}

bool DecodeActisenseAscii(const std::string& line, N2kMessage& out, std::string& error) {
    out = N2kMessage{};
    error.clear();

    std::string trimmed = line;
    while (!trimmed.empty() && (trimmed.back() == '\r' || trimmed.back() == '\n'))
        trimmed.pop_back();

    if (trimmed.empty() || trimmed[0] != 'A') return false;

    std::istringstream iss(trimmed);
    std::vector<std::string> f;
    std::string tok;
    while (iss >> tok) f.push_back(tok);
    if (f.size() < 4) {
        error = "incomplete Actisense N2K ASCII: expected at least 4 fields, got " + std::to_string(f.size());
        return true;
    }

    if (f[0].size() < 2 || f[0][0] != 'A') {
        error = "field 0 must start with Actisense message type 'A'";
        return true;
    }
    out.timestamp = f[0].substr(1);
    if (!IsActisenseTime(out.timestamp)) {
        error = "field 0 timestamp must be hhmmss or hhmmss.ddd";
        return true;
    }

    if (f[1].size() != 5 || !IsHex(f[1])) {
        error = "field 1 (<SS><DD><P>) must be 5 hex digits";
        return true;
    }
    uint8_t src, dst;
    if (!ParseHexByte(f[1].substr(0, 2), src) ||
        !ParseHexByte(f[1].substr(2, 2), dst)) {
        error = "field 1 source/destination must be hex bytes";
        return true;
    }
    char* pend = nullptr;
    long prio = std::strtol(f[1].substr(4, 1).c_str(), &pend, 16);
    if (!pend || *pend != '\0' || prio < 0 || prio > 7) {
        error = "field 1 priority must be hex 0..7";
        return true;
    }
    out.source = src;
    out.destination = dst;
    out.priority = static_cast<uint8_t>(prio);

    if (f[2].size() < 5 || !IsHex(f[2])) {
        error = "field 2 (PGN) must be at least 5 hex digits";
        return true;
    }
    out.pgn = static_cast<uint32_t>(std::strtoul(f[2].c_str(), nullptr, 16)) & 0x3ffffu;

    if ((f[3].size() % 2) != 0 || !IsHex(f[3])) {
        error = "field 3 (payload) must be an even number of hex digits";
        return true;
    }
    for (size_t i = 0; i < f[3].size(); i += 2) {
        uint8_t b;
        ParseHexByte(f[3].substr(i, 2), b);
        out.data.push_back(b);
    }
    return true;
}

bool IsGeneratedN2kPgn(uint32_t pgn) {
    static const std::set<uint32_t> generated = {
        kPgnSystemTime, kPgnHeading, kPgnWaterSpeed, kPgnPositionRapid,
        kPgnCogSogRapid, kPgnGnssPosition, kPgnWindData,
        kPgnAisClassAPosition, kPgnAisClassBPosition,
        kPgnAisClassAStatic, kPgnAisSarAircraft, kPgnAisSafetyBroadcast,
        kPgnAisClassBStaticA, kPgnAisClassBStaticB
    };
    return generated.count(pgn) != 0;
}

std::vector<std::string> BuildN2kOwnshipMessages(const OwnshipState& s, const std::tm& utc) {
    std::vector<std::string> out;
    {
        std::vector<uint8_t> d;
        U8(d, 0);
        U16(d, Angle(s.heading));
        U16(d, 0xffff); // magnetic deviation unavailable
        U16(d, 0xffff); // magnetic variation unavailable
        U8(d, 0xfc);    // true reference + reserved bits set
        out.push_back(EncodeActisenseAscii(kPgnHeading, d));
    }
    {
        std::vector<uint8_t> d;
        U8(d, 0);
        U16(d, Speed(s.speedThroughWater));
        U16(d, Speed(s.sog));
        U8(d, 1);       // paddle wheel
        U16(d, 0xfff0); // speed direction 0 (forward) + reserved bits set
        out.push_back(EncodeActisenseAscii(kPgnWaterSpeed, d));
    }
    {
        std::vector<uint8_t> d;
        I32(d, LatLon(s.latitude)); I32(d, LatLon(s.longitude));
        out.push_back(EncodeActisenseAscii(kPgnPositionRapid, d));
    }
    {
        std::vector<uint8_t> d;
        U8(d, 0);
        U8(d, 0xfc);    // true reference + reserved bits set
        U16(d, Angle(s.cog));
        U16(d, Speed(s.sog));
        U16(d, 0xffff); // reserved / sequence extension
        out.push_back(EncodeActisenseAscii(kPgnCogSogRapid, d));
    }
    out.push_back(EncodeActisenseAscii(kPgnGnssPosition, OwnshipGnss(s)));
    out.push_back(EncodeActisenseAscii(kPgnSystemTime, OwnshipSystemTime(utc)));
    {
        std::vector<uint8_t> d;
        U8(d, 0); U16(d, Speed(s.appWindSpeed)); U16(d, Angle(s.appWindAngle)); U8(d, 2);
        out.push_back(EncodeActisenseAscii(kPgnWindData, d));
    }
    {
        std::vector<uint8_t> d;
        U8(d, 0); U16(d, Speed(s.trueWindSpeed)); U16(d, Angle(s.trueWindDir)); U8(d, 3);
        out.push_back(EncodeActisenseAscii(kPgnWindData, d));
    }
    return out;
}

std::vector<std::string> BuildN2kAisDynamicMessages(const AisDynamic& a) {
    std::vector<uint8_t> d;
    if (IsSarKind(a.kind)) {
        U8(d, AisMsgRepeatByte(9));
        U32(d, a.mmsi);
        I32(d, LatLon(a.longitude));
        I32(d, LatLon(a.latitude));
        U8(d, AisTimestampByte(a.timestamp));
        U16(d, Angle(a.cog));
        U16(d, Speed(a.sog));
        U24(d, 0); // communication state unavailable
        U16(d, static_cast<uint16_t>(std::clamp(a.altitudeMeters, 0, 4094)));
        U8(d, 0); // altitude sensor GNSS, reserved clear
        U8(d, 0); // sequence id
        U16(d, 0xffff);
        return { EncodeActisenseAscii(kPgnAisSarAircraft, d, kN2kAisSource) };
    }

    U8(d, AisMsgRepeatByte(a.classA ? 1 : 18));
    U32(d, a.mmsi);
    I32(d, LatLon(a.longitude));
    I32(d, LatLon(a.latitude));
    U8(d, AisTimestampByte(a.timestamp));
    U16(d, Angle(a.cog));
    U16(d, Speed(a.sog));
    U24(d, 0); // communication state unavailable
    U16(d, Angle(a.heading));
    if (a.classA) {
        I16(d, RotRaw(a.rot));
        U8(d, 0xc0); // navigation status underway, special manoeuvre no, reserved set
        U8(d, 0xf8); // spare/reserved set
        U8(d, 0);    // sequence id
    } else {
        U8(d, 0xff); // regional application unavailable
        U8(d, 0xff); // unit/display/DSC/band/mode/state unavailable
        U16(d, 0xffff); // reserved
    }
    return { EncodeActisenseAscii(a.classA ? kPgnAisClassAPosition : kPgnAisClassBPosition,
                                  d, kN2kAisSource) };
}

std::string BuildN2kSafetyBroadcast(uint32_t mmsi, const std::string& text) {
    std::vector<uint8_t> d;
    U8(d, AisMsgRepeatByte(14));
    U32(d, mmsi);
    FixedString(d, text, std::min<size_t>(text.size(), 32));
    U8(d, 0xff); // transceiver information unavailable
    U8(d, 0);    // sequence id
    return EncodeActisenseAscii(kPgnAisSafetyBroadcast, d, kN2kAisSource);
}

std::vector<std::string> BuildN2kAisStaticMessages(const AisStatic& s) {
    std::vector<std::string> out;
    if (s.classA) {
        std::vector<uint8_t> d;
        U8(d, AisMsgRepeatByte(5));
        U32(d, s.mmsi);
        U32(d, s.imo);
        FixedString(d, s.callsign, 7);
        FixedString(d, s.name, 20);
        U8(d, s.shipType);
        U16(d, Decimeters(s.dimBow + s.dimStern));
        U16(d, Decimeters(s.dimPort + s.dimStarboard));
        U16(d, Decimeters(s.dimStarboard)); // reference point from starboard
        U16(d, Decimeters(s.dimBow));       // reference point from bow
        U16(d, 0xffff); // ETA date unavailable
        U32(d, 0xffffffffu); // ETA time unavailable
        U16(d, 0xffff); // draft unavailable
        FixedString(d, "", 20); // destination unavailable
        U16(d, 0xffff); // AIS version/GNSS/DTE/reserved unavailable
        out.push_back(EncodeActisenseAscii(kPgnAisClassAStatic, d, kN2kAisSource));
    } else {
        std::vector<uint8_t> a;
        U8(a, AisMsgRepeatByte(24));
        U32(a, s.mmsi);
        FixedString(a, s.name, 20);
        U8(a, 0xff); // transceiver information/reserved unavailable
        U8(a, 0);    // sequence id
        out.push_back(EncodeActisenseAscii(kPgnAisClassBStaticA, a, kN2kAisSource));
        std::vector<uint8_t> b;
        U8(b, AisMsgRepeatByte(24));
        U32(b, s.mmsi);
        U8(b, s.shipType);
        FixedString(b, "HMV", 7); // vendor id / model / serial placeholder
        FixedString(b, s.callsign, 7);
        U16(b, Decimeters(s.dimBow + s.dimStern));
        U16(b, Decimeters(s.dimPort + s.dimStarboard));
        U16(b, Decimeters(s.dimStarboard)); // reference point from starboard
        U16(b, Decimeters(s.dimBow));       // reference point from bow
        U32(b, 0xffffffffu); // mothership MMSI unavailable
        U16(b, 0xffff); // part number / reserved unavailable
        U8(b, 0);       // sequence id
        out.push_back(EncodeActisenseAscii(kPgnAisClassBStaticB, b, kN2kAisSource));
    }
    return out;
}

bool ParseN2kAutopilotInput(const std::string& line, ApInput& out) {
    out = ApInput{};
    N2kMessage msg;
    std::string err;
    if (!DecodeActisenseAscii(line, msg, err)) return false;
    if (!err.empty() && msg.pgn == 0) {
        std::istringstream iss(line);
        std::vector<std::string> f;
        std::string tok;
        while (iss >> tok) f.push_back(tok);
        if (f.size() >= 3 && IsHex(f[2]))
            msg.pgn = static_cast<uint32_t>(std::strtoul(f[2].c_str(), nullptr, 16)) & 0x3ffffu;
    }
    if (msg.pgn != kPgnCrossTrackError && msg.pgn != kPgnTrackControl &&
        msg.pgn != kPgnNavigationData) {
        return false;
    }
    if (!err.empty()) {
        char label[32];
        std::snprintf(label, sizeof(label), "N2K %u", msg.pgn);
        out.formatter = label;
        out.errors.push_back(err);
        return true;
    }
    if (msg.pgn == kPgnCrossTrackError) ParseCrossTrack(msg, out);
    else if (msg.pgn == kPgnTrackControl) ParseTrackControl(msg, out);
    else ParseNavigation(msg, out);
    return true;
}

std::string EncodeN2kCrossTrackError(double xteNm, char steer) {
    std::vector<uint8_t> d;
    U8(d, 0);
    U8(d, 0);
    double meters = std::fabs(xteNm) * kMetersPerNm;
    int32_t cm = static_cast<int32_t>(std::llround(meters * 100.0));
    if (steer == 'L' || steer == 'l') cm = -cm;
    I32(d, cm);
    return EncodeActisenseAscii(kPgnCrossTrackError, d);
}

std::string EncodeN2kTrackControl(double headingToSteer, double bearingToDest) {
    std::vector<uint8_t> d;
    U8(d, 0);
    U16(d, Angle(headingToSteer));
    U16(d, Angle(bearingToDest));
    U8(d, 0);
    return EncodeActisenseAscii(kPgnTrackControl, d);
}

std::string EncodeN2kNavigationData(double destLat, double destLon,
                                    double bearingToDest, double rangeNm) {
    std::vector<uint8_t> d;
    U8(d, 0);
    U32(d, static_cast<uint32_t>(std::llround(std::max(0.0, rangeNm) * kMetersPerNm * 100.0)));
    U16(d, 0xffff); // bearing origin to destination unavailable
    U16(d, Angle(bearingToDest));
    I32(d, LatLon(destLat));
    I32(d, LatLon(destLon));
    U8(d, 0);
    return EncodeActisenseAscii(kPgnNavigationData, d);
}

} // namespace nmea
