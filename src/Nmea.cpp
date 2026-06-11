// Nmea.cpp - NMEA 0183 sentence construction for ownship data.
#include "Nmea.h"

#include <cstdio>
#include <cstdarg>
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

} // namespace nmea
