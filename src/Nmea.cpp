// Nmea.cpp - NMEA 0183 sentence construction for ownship data.
#include "Nmea.h"

#include <cstdio>
#include <cstdarg>
#include <cctype>
#include <cmath>

namespace nmea {

unsigned char Checksum(const std::string& body) {
    unsigned char cs = 0;
    for (char c : body) cs ^= static_cast<unsigned char>(c);
    return cs;
}

std::string Frame(char start, const std::string& body) {
    char tail[8];
    std::snprintf(tail, sizeof(tail), "*%02X\r\n", Checksum(body));
    std::string out;
    out.reserve(body.size() + 8);
    out.push_back(start);
    out += body;
    out += tail;
    return out;
}

namespace {

// Format latitude as ddmm.mmmm and the N/S hemisphere character.
std::string FormatLat(double lat, char& hemi) {
    hemi = (lat >= 0.0) ? 'N' : 'S';
    double a = std::fabs(lat);
    int deg = static_cast<int>(a);
    double min = (a - deg) * 60.0;
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%02d%07.4f", deg, min);
    return buf;
}

// Format longitude as dddmm.mmmm and the E/W hemisphere character.
std::string FormatLon(double lon, char& hemi) {
    hemi = (lon >= 0.0) ? 'E' : 'W';
    double a = std::fabs(lon);
    int deg = static_cast<int>(a);
    double min = (a - deg) * 60.0;
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%03d%07.4f", deg, min);
    return buf;
}

std::string F(const char* fmt, ...) {
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return buf;
}

} // namespace

std::vector<std::string> BuildOwnshipSentences(const OwnshipState& s, const std::tm& utc) {
    std::vector<std::string> out;

    char latH, lonH;
    const std::string lat = FormatLat(s.latitude, latH);
    const std::string lon = FormatLon(s.longitude, lonH);

    const std::string timeStr = F("%02d%02d%02d.00", utc.tm_hour, utc.tm_min, utc.tm_sec);
    const std::string dateStr = F("%02d%02d%02d", utc.tm_mday, utc.tm_mon + 1, (utc.tm_year + 1900) % 100);

    // GGA - fix data.
    out.push_back(Frame('$', F("GPGGA,%s,%s,%c,%s,%c,1,08,0.9,0.0,M,0.0,M,,",
        timeStr.c_str(), lat.c_str(), latH, lon.c_str(), lonH)));

    // RMC - recommended minimum.
    out.push_back(Frame('$', F("GPRMC,%s,A,%s,%c,%s,%c,%.1f,%.1f,%s,,,A",
        timeStr.c_str(), lat.c_str(), latH, lon.c_str(), lonH,
        s.sog, s.cog, dateStr.c_str())));

    // VTG - course and speed over ground.
    out.push_back(Frame('$', F("GPVTG,%.1f,T,,M,%.1f,N,%.1f,K,A",
        s.cog, s.sog, s.sog * 1.852)));

    // GLL - geographic position.
    out.push_back(Frame('$', F("GPGLL,%s,%c,%s,%c,%s,A",
        lat.c_str(), latH, lon.c_str(), lonH, timeStr.c_str())));

    // VHW - water speed and heading.
    out.push_back(Frame('$', F("VWVHW,%.1f,T,,M,%.1f,N,%.1f,K",
        s.heading, s.speedThroughWater, s.speedThroughWater * 1.852)));

    // MWV - apparent (relative) wind.
    out.push_back(Frame('$', F("WIMWV,%.1f,R,%.1f,N,A",
        s.appWindAngle, s.appWindSpeed)));

    // MWV - true wind (relative to vessel).
    {
        double twAngle = s.trueWindDir - s.heading;
        twAngle = std::fmod(twAngle, 360.0);
        if (twAngle < 0.0) twAngle += 360.0;
        out.push_back(Frame('$', F("WIMWV,%.1f,T,%.1f,N,A",
            twAngle, s.trueWindSpeed)));
    }

    // MWD - true wind direction and speed.
    out.push_back(Frame('$', F("WIMWD,%.1f,T,,M,%.1f,N,%.1f,M",
        s.trueWindDir, s.trueWindSpeed, s.trueWindSpeed * 0.514444)));

    return out;
}

// ---- Incoming sentence parsing -------------------------------------------

namespace {
// Position of the '*' that introduces the checksum, or npos.
size_t StarPos(const std::string& line) {
    return line.find('*');
}
} // namespace

std::string SentenceFormatter(const std::string& line) {
    if (line.size() < 2 || (line[0] != '$' && line[0] != '!')) return "";
    size_t comma = line.find(',');
    std::string address = line.substr(1, (comma == std::string::npos)
                                            ? std::string::npos : comma - 1);
    if (address.size() < 3) return "";
    std::string fmt = address.substr(address.size() - 3);
    for (char& c : fmt) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return fmt;
}

bool VerifyChecksum(const std::string& line) {
    size_t star = StarPos(line);
    if (star == std::string::npos) return true; // nothing to verify
    if (line.empty() || (line[0] != '$' && line[0] != '!')) return false;
    if (star + 2 >= line.size()) return false;

    unsigned char cs = 0;
    for (size_t i = 1; i < star; ++i) cs ^= static_cast<unsigned char>(line[i]);

    char expected[4];
    std::snprintf(expected, sizeof(expected), "%02X", cs);
    char a = static_cast<char>(std::toupper(static_cast<unsigned char>(line[star + 1])));
    char b = static_cast<char>(std::toupper(static_cast<unsigned char>(line[star + 2])));
    return a == expected[0] && b == expected[1];
}

std::vector<std::string> SplitFields(const std::string& line) {
    std::string body = line;
    // Strip trailing CR/LF.
    while (!body.empty() && (body.back() == '\r' || body.back() == '\n')) body.pop_back();
    // Strip checksum.
    size_t star = body.find('*');
    if (star != std::string::npos) body = body.substr(0, star);

    std::vector<std::string> fields;
    size_t start = 0;
    while (true) {
        size_t comma = body.find(',', start);
        if (comma == std::string::npos) {
            fields.push_back(body.substr(start));
            break;
        }
        fields.push_back(body.substr(start, comma - start));
        start = comma + 1;
    }
    return fields;
}

bool ParseLatLon(const std::string& value, const std::string& hemi, double& out) {
    if (value.empty()) return false;
    double v;
    try { v = std::stod(value); } catch (...) { return false; }
    int deg = static_cast<int>(v / 100.0);
    double min = v - deg * 100.0;
    out = deg + min / 60.0;
    if (!hemi.empty() && (hemi[0] == 'S' || hemi[0] == 'W')) out = -out;
    return true;
}

} // namespace nmea
