// N2k.h - NMEA 2000 PGN payload encoding over Actisense N2K ASCII lines.
#pragma once

#include "Ais.h"
#include "ApInput.h"
#include "Nmea.h"

#include <cstdint>
#include <ctime>
#include <string>
#include <vector>

namespace nmea {

// The app transports NMEA 2000 PGN payloads as Actisense N2K ASCII lines:
//   Ahhmmss.ddd <SS><DD><P> <PPPPP> <payload hex>\r\n
// SS=source, DD=destination, P=priority, PPPPP=PGN, all hexadecimal.
struct N2kMessage {
    uint32_t pgn = 0;
    std::string timestamp; // hhmmss or hhmmss.ddd, as received
    uint8_t source = 35;
    uint8_t destination = 0xff;
    uint8_t priority = 3;
    std::vector<uint8_t> data;
};

constexpr uint8_t kN2kOwnshipSource = 35;
constexpr uint8_t kN2kAisSource = 36;

// Common PGNs generated or accepted by the app.
constexpr uint32_t kPgnSystemTime = 126992;
constexpr uint32_t kPgnHeading = 127250;
constexpr uint32_t kPgnTrackControl = 127237;
constexpr uint32_t kPgnWaterSpeed = 128259;
constexpr uint32_t kPgnPositionRapid = 129025;
constexpr uint32_t kPgnCogSogRapid = 129026;
constexpr uint32_t kPgnGnssPosition = 129029;
constexpr uint32_t kPgnCrossTrackError = 129283;
constexpr uint32_t kPgnNavigationData = 129284;
constexpr uint32_t kPgnAisClassAPosition = 129038;
constexpr uint32_t kPgnAisClassBPosition = 129039;
constexpr uint32_t kPgnAisClassAStatic = 129794;
constexpr uint32_t kPgnAisSarAircraft = 129798;
constexpr uint32_t kPgnAisSafetyBroadcast = 129802;
constexpr uint32_t kPgnAisClassBStaticA = 129809;
constexpr uint32_t kPgnAisClassBStaticB = 129810;
constexpr uint32_t kPgnWindData = 130306;

std::string EncodeActisenseAscii(uint32_t pgn, const std::vector<uint8_t>& data,
                                 uint8_t source = kN2kOwnshipSource,
                                 uint8_t destination = 0xff,
                                 uint8_t priority = 0xff);

bool DecodeActisenseAscii(const std::string& line, N2kMessage& out, std::string& error);
bool IsGeneratedN2kPgn(uint32_t pgn);

std::vector<std::string> BuildN2kOwnshipMessages(const OwnshipState& s, const std::tm& utc);
std::vector<std::string> BuildN2kAisDynamicMessages(const AisDynamic& d);
std::vector<std::string> BuildN2kAisStaticMessages(const AisStatic& s);
std::string BuildN2kSafetyBroadcast(uint32_t mmsi, const std::string& text);

// Decode and validate NMEA 2000 autopilot-equivalent input PGNs into the same
// structure used by APB/RMB/XTE. Returns false if the Actisense ASCII line is
// not one of the autopilot PGNs this app understands.
bool ParseN2kAutopilotInput(const std::string& line, ApInput& out);

// Test helpers for constructing well-formed inbound autopilot PGNs.
std::string EncodeN2kCrossTrackError(double xteNm, char steer);
std::string EncodeN2kTrackControl(double headingToSteer, double bearingToDest);
std::string EncodeN2kNavigationData(double destLat, double destLon,
                                    double bearingToDest, double rangeNm);

} // namespace nmea
