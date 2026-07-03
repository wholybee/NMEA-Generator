// Ais.h - AIVDM/AIS binary message encoding (NMEA 0183 AIS).
#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace nmea {

enum class AisTargetKind {
    ClassA = 0,
    ClassB = 1,
    SarFixedWing = 2,
    SarHelicopter = 3
};

bool IsSarKind(AisTargetKind kind);

// Static identity of an AIS target. Generated pseudo-randomly per target.
struct AisStatic {
    uint32_t mmsi = 0;
    uint32_t imo = 0;
    std::string name;       // up to 20 chars
    std::string callsign;   // up to 7 chars
    uint8_t shipType = 0;   // AIS ship & cargo type
    uint16_t dimBow = 0;    // metres, reference point to bow
    uint16_t dimStern = 0;  // metres, reference point to stern
    uint8_t dimPort = 0;    // metres, reference point to port
    uint8_t dimStarboard = 0; // metres, reference point to starboard
    bool classA = true;     // true = Class A, false = Class B
    AisTargetKind kind = AisTargetKind::ClassA;
};

// Dynamic position state of an AIS target.
struct AisDynamic {
    uint32_t mmsi = 0;
    double latitude = 0.0;  // decimal degrees
    double longitude = 0.0; // decimal degrees
    double sog = 0.0;       // knots
    double cog = 0.0;       // degrees true
    double heading = 0.0;   // degrees true
    double rot = 0.0;       // degrees per minute (signed)
    int altitudeMeters = 0; // used by SAR aircraft position reports
    int timestamp = 0;      // UTC second 0..59
    bool classA = true;
    AisTargetKind kind = AisTargetKind::ClassA;
};

// Encode a dynamic position report. Returns one !AIVDM sentence (type 1 for
// Class A, type 18 for Class B), channel A/B alternating by 'channel'.
std::string EncodePositionReport(const AisDynamic& d, char channel);

// Encode an AIS safety-related broadcast (message 14).
std::string EncodeSafetyBroadcast(uint32_t mmsi, const std::string& text, char channel);

// Encode static/voyage data. Class A -> a 2-part type 5 message; Class B ->
// a 2-part type 24 (Part A + Part B). Returns one or more !AIVDM sentences.
std::vector<std::string> EncodeStaticReport(const AisStatic& s, char channel,
                                            int& seqId);

// ---- Exposed for testing -------------------------------------------------

// Append the low 'bits' bits of value (MSB first) to a growing bit string.
void AppendUInt(std::string& bits, uint64_t value, int bits_n);
void AppendInt(std::string& bits, int64_t value, int bits_n);
// Append a string as 6-bit AIS characters, padded/truncated to 'chars'.
void AppendString(std::string& bits, const std::string& text, int chars);
// Convert a bit string to the 6-bit ASCII armoured payload; sets fill bits.
std::string ArmorPayload(const std::string& bits, int& fillBits);

} // namespace nmea
