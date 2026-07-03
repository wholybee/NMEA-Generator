// Ais.cpp - AIVDM/AIS binary message encoding (NMEA 0183 AIS).
#include "Ais.h"
#include "Nmea.h"

#include <cmath>
#include <cstdio>
#include <cctype>
#include <algorithm>

namespace nmea {

bool IsSarKind(AisTargetKind kind) {
    return kind == AisTargetKind::SarFixedWing || kind == AisTargetKind::SarHelicopter;
}

void AppendUInt(std::string& bits, uint64_t value, int bits_n) {
    for (int i = bits_n - 1; i >= 0; --i) {
        bits.push_back(((value >> i) & 1ULL) ? '1' : '0');
    }
}

void AppendInt(std::string& bits, int64_t value, int bits_n) {
    // Two's complement of the requested width.
    const uint64_t mask = (bits_n >= 64) ? ~0ULL : ((1ULL << bits_n) - 1ULL);
    AppendUInt(bits, static_cast<uint64_t>(value) & mask, bits_n);
}

// Map an ASCII character to its 6-bit AIS value (table per ITU-R M.1371).
static uint8_t SixBit(char c) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    // The AIS 6-bit character set: @A..Z[\]^_ space ! .. up to '?' (0..63).
    // Practically: '@'(0) A-Z(1-26) then punctuation, space(32), digits, etc.
    if (c >= '@' && c <= '_') return static_cast<uint8_t>(c - '@');      // 0..31
    if (c >= ' ' && c <= '?') return static_cast<uint8_t>(c - ' ' + 32); // 32..63
    return 0; // treat unknown as '@'
}

void AppendString(std::string& bits, const std::string& text, int chars) {
    for (int i = 0; i < chars; ++i) {
        char c = (i < static_cast<int>(text.size())) ? text[i] : '@'; // '@' = pad
        AppendUInt(bits, SixBit(c), 6);
    }
}

std::string ArmorPayload(const std::string& bits, int& fillBits) {
    std::string padded = bits;
    fillBits = (6 - static_cast<int>(padded.size() % 6)) % 6;
    padded.append(fillBits, '0');

    std::string out;
    out.reserve(padded.size() / 6);
    for (size_t i = 0; i < padded.size(); i += 6) {
        uint8_t v = 0;
        for (int b = 0; b < 6; ++b) {
            v = static_cast<uint8_t>((v << 1) | (padded[i + b] - '0'));
        }
        // 6-bit to ASCII armour.
        v += 48;
        if (v > 87) v += 8;
        out.push_back(static_cast<char>(v));
    }
    return out;
}

namespace {

// Build one or more !AIVDM sentences from a finished bit string. Splits the
// armoured payload across fragments if it exceeds the single-sentence limit.
std::vector<std::string> WrapAivdm(const std::string& bits, char channel, int& seqId) {
    int fill = 0;
    const std::string payload = ArmorPayload(bits, fill);

    // Max payload chars per sentence (keeps total sentence under 82 chars).
    const size_t kMax = 60;
    std::vector<std::string> out;

    if (payload.size() <= kMax) {
        char body[160];
        std::snprintf(body, sizeof(body), "AIVDM,1,1,,%c,%s,%d",
                      channel, payload.c_str(), fill);
        out.push_back(Frame('!', body));
        return out;
    }

    const int total = static_cast<int>((payload.size() + kMax - 1) / kMax);
    const int seq = (seqId++ % 10);
    for (int i = 0; i < total; ++i) {
        const std::string frag = payload.substr(i * kMax, kMax);
        const int fillBits = (i == total - 1) ? fill : 0;
        char body[160];
        std::snprintf(body, sizeof(body), "AIVDM,%d,%d,%d,%c,%s,%d",
                      total, i + 1, seq, channel, frag.c_str(), fillBits);
        out.push_back(Frame('!', body));
    }
    return out;
}

// Encode rate of turn into the AIS 8-bit signed field (ITU-R M.1371).
int EncodeRot(double degPerMin) {
    if (degPerMin == 0.0) return 0;
    double v = 4.733 * std::sqrt(std::fabs(degPerMin));
    int r = static_cast<int>(std::round(v));
    if (r > 126) r = 126;
    return (degPerMin < 0.0) ? -r : r;
}

// Longitude/latitude to 1/10000-minute integer units used by AIS.
int32_t EncodeLon(double lon) {
    return static_cast<int32_t>(std::round(lon * 600000.0));
}
int32_t EncodeLat(double lat) {
    return static_cast<int32_t>(std::round(lat * 600000.0));
}

} // namespace

std::string EncodePositionReport(const AisDynamic& d, char channel) {
    std::string bits;

    const int sog = static_cast<int>(std::round(std::min(d.sog, 102.2) * 10.0));
    const int cog = static_cast<int>(std::round(d.cog * 10.0)) % 3600;
    const int hdg = (d.heading >= 0.0 && d.heading < 360.0)
                        ? static_cast<int>(std::round(d.heading)) : 511;

    if (IsSarKind(d.kind)) {
        const int altitude = std::clamp(d.altitudeMeters, 0, 4094);
        AppendUInt(bits, 9, 6);              // message type 9: SAR aircraft
        AppendUInt(bits, 0, 2);              // repeat indicator
        AppendUInt(bits, d.mmsi, 30);        // MMSI
        AppendUInt(bits, altitude, 12);      // altitude, metres
        AppendUInt(bits, sog, 10);           // speed over ground
        AppendUInt(bits, 0, 1);              // position accuracy
        AppendInt(bits, EncodeLon(d.longitude), 28);
        AppendInt(bits, EncodeLat(d.latitude), 27);
        AppendUInt(bits, cog, 12);           // course over ground
        AppendUInt(bits, d.timestamp, 6);    // UTC second
        AppendUInt(bits, 0, 1);              // altitude sensor: GNSS
        AppendUInt(bits, 0, 7);              // reserved
        AppendUInt(bits, 0, 1);              // DTE
        AppendUInt(bits, 0, 3);              // spare
        AppendUInt(bits, 0, 1);              // assigned mode
        AppendUInt(bits, 0, 1);              // RAIM
        AppendUInt(bits, 0, 19);             // radio status
    } else if (d.classA) {
        AppendUInt(bits, 1, 6);              // message type 1
        AppendUInt(bits, 0, 2);              // repeat indicator
        AppendUInt(bits, d.mmsi, 30);        // MMSI
        AppendUInt(bits, 0, 4);              // nav status: under way using engine
        AppendInt(bits, EncodeRot(d.rot), 8);// rate of turn
        AppendUInt(bits, sog, 10);           // speed over ground
        AppendUInt(bits, 0, 1);              // position accuracy
        AppendInt(bits, EncodeLon(d.longitude), 28);
        AppendInt(bits, EncodeLat(d.latitude), 27);
        AppendUInt(bits, cog, 12);           // course over ground
        AppendUInt(bits, hdg, 9);            // true heading
        AppendUInt(bits, d.timestamp, 6);    // UTC second
        AppendUInt(bits, 0, 2);              // maneuver indicator
        AppendUInt(bits, 0, 3);              // spare
        AppendUInt(bits, 0, 1);              // RAIM
        AppendUInt(bits, 0, 19);             // radio status
    } else {
        AppendUInt(bits, 18, 6);             // message type 18 (Class B)
        AppendUInt(bits, 0, 2);              // repeat
        AppendUInt(bits, d.mmsi, 30);        // MMSI
        AppendUInt(bits, 0, 8);              // reserved
        AppendUInt(bits, sog, 10);           // SOG
        AppendUInt(bits, 0, 1);              // position accuracy
        AppendInt(bits, EncodeLon(d.longitude), 28);
        AppendInt(bits, EncodeLat(d.latitude), 27);
        AppendUInt(bits, cog, 12);           // COG
        AppendUInt(bits, hdg, 9);            // heading
        AppendUInt(bits, d.timestamp, 6);    // UTC second
        AppendUInt(bits, 0, 2);              // reserved
        AppendUInt(bits, 1, 1);              // CS unit (1 = Class B "CS")
        AppendUInt(bits, 0, 1);              // display flag
        AppendUInt(bits, 0, 1);              // DSC flag
        AppendUInt(bits, 1, 1);              // band flag
        AppendUInt(bits, 1, 1);              // message 22 flag
        AppendUInt(bits, 0, 1);              // assigned
        AppendUInt(bits, 0, 1);              // RAIM
        AppendUInt(bits, 0, 20);             // radio status
    }

    int seq = 0;
    auto sentences = WrapAivdm(bits, channel, seq);
    return sentences.front();
}

std::string EncodeSafetyBroadcast(uint32_t mmsi, const std::string& text, char channel) {
    std::string bits;
    AppendUInt(bits, 14, 6);                 // safety-related broadcast
    AppendUInt(bits, 0, 2);                  // repeat
    AppendUInt(bits, mmsi, 30);              // source MMSI
    AppendUInt(bits, 0, 2);                  // spare
    AppendString(bits, text, static_cast<int>(std::min<size_t>(text.size(), 26)));

    int seq = 0;
    auto sentences = WrapAivdm(bits, channel, seq);
    return sentences.front();
}

std::vector<std::string> EncodeStaticReport(const AisStatic& s, char channel,
                                            int& seqId) {
    std::vector<std::string> out;

    if (s.classA) {
        // Type 5: static and voyage related data (424 bits -> 2 fragments).
        std::string bits;
        AppendUInt(bits, 5, 6);              // message type 5
        AppendUInt(bits, 0, 2);              // repeat
        AppendUInt(bits, s.mmsi, 30);        // MMSI
        AppendUInt(bits, 0, 2);              // AIS version
        AppendUInt(bits, s.imo, 30);         // IMO number
        AppendString(bits, s.callsign, 7);   // call sign (42 bits)
        AppendString(bits, s.name, 20);      // vessel name (120 bits)
        AppendUInt(bits, s.shipType, 8);     // ship type
        AppendUInt(bits, s.dimBow, 9);       // dimension to bow
        AppendUInt(bits, s.dimStern, 9);     // dimension to stern
        AppendUInt(bits, s.dimPort, 6);      // dimension to port
        AppendUInt(bits, s.dimStarboard, 6); // dimension to starboard
        AppendUInt(bits, 1, 4);              // EPFD type: GPS
        AppendUInt(bits, 0, 4);              // ETA month
        AppendUInt(bits, 0, 5);              // ETA day
        AppendUInt(bits, 24, 5);             // ETA hour (24 = n/a)
        AppendUInt(bits, 60, 6);             // ETA minute (60 = n/a)
        AppendUInt(bits, 0, 8);              // draught
        AppendString(bits, "", 20);          // destination (120 bits)
        AppendUInt(bits, 0, 1);              // DTE
        AppendUInt(bits, 0, 1);              // spare

        auto frags = WrapAivdm(bits, channel, seqId);
        out.insert(out.end(), frags.begin(), frags.end());
    } else {
        // Type 24 Part A: vessel name.
        {
            std::string bits;
            AppendUInt(bits, 24, 6);         // message type 24
            AppendUInt(bits, 0, 2);          // repeat
            AppendUInt(bits, s.mmsi, 30);    // MMSI
            AppendUInt(bits, 0, 2);          // part number A
            AppendString(bits, s.name, 20);  // vessel name
            AppendUInt(bits, 0, 8);          // spare
            int seq = 0;
            auto frag = WrapAivdm(bits, channel, seq);
            out.insert(out.end(), frag.begin(), frag.end());
        }
        // Type 24 Part B: type, dimensions, call sign.
        {
            std::string bits;
            AppendUInt(bits, 24, 6);         // message type 24
            AppendUInt(bits, 0, 2);          // repeat
            AppendUInt(bits, s.mmsi, 30);    // MMSI
            AppendUInt(bits, 1, 2);          // part number B
            AppendUInt(bits, s.shipType, 8); // ship type
            AppendString(bits, "HMV", 3);    // vendor ID (18 bits)
            AppendUInt(bits, 1, 4);          // unit model code
            AppendUInt(bits, 1, 20);         // serial number
            AppendString(bits, s.callsign, 7); // call sign
            AppendUInt(bits, s.dimBow, 9);
            AppendUInt(bits, s.dimStern, 9);
            AppendUInt(bits, s.dimPort, 6);
            AppendUInt(bits, s.dimStarboard, 6);
            AppendUInt(bits, 0, 6);          // spare
            int seq = 0;
            auto frag = WrapAivdm(bits, channel, seq);
            out.insert(out.end(), frag.begin(), frag.end());
        }
    }

    return out;
}

} // namespace nmea
