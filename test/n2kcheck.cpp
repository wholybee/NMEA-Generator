// n2kcheck.cpp - Actisense N2K ASCII transport and autopilot smoke tests.
#include "../src/N2k.h"
#include "../src/Simulation.h"
#include "../src/Network.h"

#include <winsock2.h>
#include <ws2tcpip.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "Ws2_32.lib")

using namespace nmea;

namespace {
int failures = 0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

void check(bool cond, const char* msg) {
    if (cond) std::printf("ok  : %s\n", msg);
    else { std::printf("FAIL: %s\n", msg); ++failures; }
}

void SendLine(SOCKET s, const std::string& line) {
    send(s, line.c_str(), static_cast<int>(line.size()), 0);
}

uint16_t Le16(const std::vector<uint8_t>& d, size_t off) {
    return static_cast<uint16_t>(d[off] | (d[off + 1] << 8));
}

double RawAngleDeg(uint16_t raw) {
    return raw / 10000.0 * kRadToDeg;
}

double RawSpeedKnots(uint16_t raw) {
    return (raw / 100.0) / 0.514444;
}
} // namespace

int main() {
    std::tm utc{};
    utc.tm_year = 126;
    utc.tm_mon = 5;
    utc.tm_mday = 29;
    utc.tm_yday = 179;
    utc.tm_hour = 12;
    utc.tm_min = 34;
    utc.tm_sec = 56;

    OwnshipState os;
    os.latitude = 50.8;
    os.longitude = -1.1;
    os.cog = 123.4;
    os.sog = 9.9;
    os.heading = 120.0;
    os.speedThroughWater = 9.5;
    os.appWindAngle = 45.0;
    os.appWindSpeed = 14.0;
    os.trueWindDir = 315.0;
    os.trueWindSpeed = 12.0;

    auto own = BuildN2kOwnshipMessages(os, utc);
    check(own.size() == 8, "ownship emits 8 NMEA 2000 PGN lines");

    int posRapid = 0, wind = 0, waterSpeed = 0, cogSog = 0;
    for (const auto& line : own) {
        N2kMessage m;
        std::string err;
        check(!line.empty() && line[0] == 'A', "generated line uses Actisense N2K ASCII message type");
        check(DecodeActisenseAscii(line, m, err) && err.empty(), "generated Actisense ASCII decodes");
        check(m.destination == 0xff && m.priority <= 7, "Actisense header decodes destination and priority");
        if (m.pgn == kPgnPositionRapid) ++posRapid;
        if (m.pgn == kPgnWindData) ++wind;
        if (m.pgn == kPgnWaterSpeed) {
            ++waterSpeed;
            check(m.data.size() == 8, "PGN 128259 speed payload is 8 bytes");
        }
        if (m.pgn == kPgnCogSogRapid) {
            ++cogSog;
            check(m.data.size() == 8, "PGN 129026 COG/SOG payload is 8 bytes");
        }
    }
    check(posRapid == 1, "PGN 129025 position rapid present");
    check(waterSpeed == 1, "PGN 128259 speed present");
    check(cogSog == 1, "PGN 129026 COG/SOG present");
    check(wind == 2, "PGN 130306 apparent and true wind present");

    AisStatic st = MakeStaticIdentity(0, AisTargetKind::ClassA);
    AisDynamic dyn;
    dyn.mmsi = st.mmsi;
    dyn.classA = true;
    dyn.latitude = 50.9;
    dyn.longitude = -1.0;
    dyn.sog = 8.0;
    dyn.cog = 40.0;
    dyn.heading = 41.0;
    dyn.timestamp = 12;
    auto ad = BuildN2kAisDynamicMessages(dyn);
    N2kMessage dm;
    std::string derr;
    DecodeActisenseAscii(ad[0], dm, derr);
    check(derr.empty() && dm.pgn == kPgnAisClassAPosition, "AIS Class A dynamic emits PGN 129038");
    check(dm.data.size() == 28, "PGN 129038 AIS Class A dynamic payload is 28 bytes");
    check(std::fabs(RawAngleDeg(Le16(dm.data, 14)) - 40.0) < 0.1,
          "PGN 129038 COG decodes from Canboat offset");
    check(std::fabs(RawSpeedKnots(Le16(dm.data, 16)) - 8.0) < 0.1,
          "PGN 129038 SOG decodes from Canboat offset");
    auto as = BuildN2kAisStaticMessages(st);
    N2kMessage sm;
    DecodeActisenseAscii(as[0], sm, derr);
    check(!as.empty() && sm.pgn == kPgnAisClassAStatic, "AIS Class A static emits PGN 129794");
    check(sm.data.size() == 75, "PGN 129794 AIS Class A static payload is 75 bytes");
    check(std::string(reinterpret_cast<const char*>(&sm.data[16]), st.name.size()) == st.name,
          "PGN 129794 vessel name starts at Canboat offset");

    AisStatic stb = MakeStaticIdentity(1, AisTargetKind::ClassB);
    AisDynamic dynb = dyn;
    dynb.mmsi = stb.mmsi;
    dynb.classA = false;
    auto bd = BuildN2kAisDynamicMessages(dynb);
    N2kMessage bdm;
    DecodeActisenseAscii(bd[0], bdm, derr);
    check(derr.empty() && bdm.pgn == kPgnAisClassBPosition, "AIS Class B dynamic emits PGN 129039");
    check(bdm.data.size() == 27, "PGN 129039 AIS Class B dynamic payload is 27 bytes");
    check(std::fabs(RawAngleDeg(Le16(bdm.data, 14)) - 40.0) < 0.1,
          "PGN 129039 COG decodes from Canboat offset");
    check(std::fabs(RawSpeedKnots(Le16(bdm.data, 16)) - 8.0) < 0.1,
          "PGN 129039 SOG decodes from Canboat offset");
    auto bs = BuildN2kAisStaticMessages(stb);
    N2kMessage bsm0, bsm1;
    DecodeActisenseAscii(bs[0], bsm0, derr);
    DecodeActisenseAscii(bs[1], bsm1, derr);
    check(bs.size() == 2 && bsm0.pgn == kPgnAisClassBStaticA &&
              bsm1.pgn == kPgnAisClassBStaticB,
          "AIS Class B static emits PGNs 129809 and 129810");
    check(bsm0.data.size() == 27, "PGN 129809 AIS Class B static A payload is 27 bytes");
    check(bsm1.data.size() == 35, "PGN 129810 AIS Class B static B payload is 35 bytes");
    check(std::string(reinterpret_cast<const char*>(&bsm0.data[5]), stb.name.size()) == stb.name,
          "PGN 129809 vessel name starts at Canboat offset");

    AisDynamic sar = dyn;
    sar.kind = AisTargetKind::SarHelicopter;
    sar.classA = false;
    sar.mmsi = 111366777;
    sar.altitudeMeters = 150;
    auto sd = BuildN2kAisDynamicMessages(sar);
    N2kMessage sdm;
    DecodeActisenseAscii(sd[0], sdm, derr);
    check(derr.empty() && sdm.pgn == kPgnAisSarAircraft, "SAR aircraft emits PGN 129798");
    check(sdm.data.size() >= 24, "PGN 129798 SAR payload has expected data");
    check(sdm.data[0] == 9, "PGN 129798 carries AIS message 9 id");

    std::string mobText = BuildN2kSafetyBroadcast(972366001u, "MOB ACTIVE");
    N2kMessage mobMsg;
    DecodeActisenseAscii(mobText, mobMsg, derr);
    check(derr.empty() && mobMsg.pgn == kPgnAisSafetyBroadcast,
          "MOB safety broadcast emits PGN 129802");
    check(!mobMsg.data.empty() && mobMsg.data[0] == 14,
          "PGN 129802 carries AIS message 14 id");

    ApInput in;
    std::string xte = EncodeN2kCrossTrackError(0.25, 'L');
    check(ParseN2kAutopilotInput(xte, in) && in.ok(), "PGN 129283 XTE validates");
    check(in.hasXte && std::fabs(in.xteNm - 0.25) < 0.01 && in.xteDir == 'L',
          "PGN 129283 XTE decodes");

    std::string nav = EncodeN2kNavigationData(50.5, -0.5, 45.0, 12.5);
    check(ParseN2kAutopilotInput(nav, in) && in.ok(), "PGN 129284 navigation validates");
    check(in.hasDest && std::fabs(in.destLat - 50.5) < 1e-6 &&
              std::fabs(in.destLon + 0.5) < 1e-6,
          "PGN 129284 destination decodes");
    check(in.hasBearingToDest && std::fabs(in.bearingToDest - 45.0) < 0.1,
          "PGN 129284 bearing decodes");
    check(in.hasRange && std::fabs(in.rangeNm - 12.5) < 0.01,
          "PGN 129284 range decodes");

    std::string bad = "A123456.000 23FF3 1F904 00XX\r\n";
    ParseN2kAutopilotInput(bad, in);
    check(!in.ok() && in.ErrorText().find("payload") != std::string::npos,
          "bad N2K autopilot payload is reported");

    // End-to-end: feed PGN 129284 over TCP and confirm the simulator leaves
    // pattern mode and steers toward the waypoint.
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);

    constexpr unsigned short port = 10115;
    NetworkServer net;
    std::string err;
    check(net.Start(port, port, err), "network server starts for N2K autopilot test");

    std::atomic<double> lastLat{0.0}, lastLon{0.0};
    Simulation sim([&](const std::string& line) {
        net.Send(line);
        N2kMessage m;
        std::string e;
        if (DecodeActisenseAscii(line, m, e) && e.empty() && m.pgn == kPgnPositionRapid && m.data.size() >= 8) {
            int32_t lat = static_cast<int32_t>(
                static_cast<uint32_t>(m.data[0]) |
                (static_cast<uint32_t>(m.data[1]) << 8) |
                (static_cast<uint32_t>(m.data[2]) << 16) |
                (static_cast<uint32_t>(m.data[3]) << 24));
            int32_t lon = static_cast<int32_t>(
                static_cast<uint32_t>(m.data[4]) |
                (static_cast<uint32_t>(m.data[5]) << 8) |
                (static_cast<uint32_t>(m.data[6]) << 16) |
                (static_cast<uint32_t>(m.data[7]) << 24));
            lastLat.store(lat / 10000000.0);
            lastLon.store(lon / 10000000.0);
        }
    });
    net.SetReceiveSink([&](const std::string& line) { sim.OnIncomingSentence(line); });
    SimConfig cfg;
    cfg.protocol = ProtocolMode::Nmea2000;
    cfg.ownship.centreLat = 50.0;
    cfg.ownship.centreLon = -1.0;
    cfg.ownship.speed = 600.0;
    sim.SetConfig(cfg);
    sim.Start();

    std::this_thread::sleep_for(std::chrono::seconds(2));
    double beforeLat = lastLat.load();
    double beforeLon = lastLon.load();

    SOCKET cs = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    check(connect(cs, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0, "TCP client connects");
    SendLine(cs, EncodeN2kNavigationData(50.5, -0.5, 45.0, 30.0));

    std::this_thread::sleep_for(std::chrono::seconds(4));
    check(sim.AutopilotEngaged(), "autopilot engages from PGN 129284");
    check(lastLat.load() > beforeLat, "N2K autopilot increases latitude");
    check(lastLon.load() > beforeLon, "N2K autopilot increases longitude");

    closesocket(cs);
    sim.Stop();
    net.Stop();
    WSACleanup();

    if (failures) {
        std::printf("\nFAILURES PRESENT (%d failures)\n", failures);
        return 1;
    }
    std::printf("\nALL PASSED (0 failures)\n");
    return 0;
}
