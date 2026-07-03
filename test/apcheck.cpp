// apcheck.cpp - End-to-end test: feed an RMB over TCP and confirm the ownship
// switches from its pattern to steering toward the destination waypoint.
#include "../src/Simulation.h"
#include "../src/Network.h"
#include "../src/Nmea.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdio>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>

#pragma comment(lib, "Ws2_32.lib")

using namespace nmea;

static std::mutex g_m;
static double g_lat = 0, g_lon = 0, g_cog = 0;
static bool g_haveFix = false;

// Pull lat/lon/cog out of generated RMC sentences.
static void CaptureRmc(const std::string& line) {
    if (SentenceFormatter(line) != "RMC") return;
    auto f = SplitFields(line);
    if (f.size() < 9) return;
    double lat, lon;
    if (!ParseLatLon(f[3], f[4], lat)) return;
    if (!ParseLatLon(f[5], f[6], lon)) return;
    std::lock_guard<std::mutex> lk(g_m);
    g_lat = lat; g_lon = lon;
    try { g_cog = std::stod(f[8]); } catch (...) {}
    g_haveFix = true;
}

int main() {
    WSADATA w; WSAStartup(MAKEWORD(2, 2), &w);

    NetworkServer net;
    std::string err;
    if (!net.Start(10998, 10998, err)) { printf("server failed: %s\n", err.c_str()); return 1; }

    Simulation sim([&](const std::string& line) { net.Send(line); CaptureRmc(line); });
    net.SetReceiveSink([&](const std::string& line) { sim.OnIncomingSentence(line); });

    SimConfig cfg;
    cfg.ownship.centreLat = 50.0;
    cfg.ownship.centreLon = -1.0;
    cfg.ownship.speed = 30.0;
    cfg.ownship.shape = Shape::Circle;
    sim.SetConfig(cfg);
    sim.Start();

    // TCP client.
    SOCKET c = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(10998);
    InetPtonA(AF_INET, "127.0.0.1", &a.sin_addr);
    int conn = -1;
    for (int i = 0; i < 20 && conn != 0; ++i) {
        conn = connect(c, (sockaddr*)&a, sizeof(a));
        if (conn != 0) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (conn != 0) { printf("connect failed\n"); return 1; }

    int fail = 0;
    auto check = [&](bool cond, const char* msg) {
        printf("%s: %s\n", cond ? "ok  " : "FAIL", msg);
        if (!cond) ++fail;
    };

    // Let the pattern run, then sample position + engaged state.
    std::this_thread::sleep_for(std::chrono::seconds(3));
    check(!sim.AutopilotEngaged(), "autopilot disengaged before any APB/RMB/XTE");
    double lat0, lon0;
    { std::lock_guard<std::mutex> lk(g_m); lat0 = g_lat; lon0 = g_lon; }
    printf("     pattern pos: %.5f, %.5f\n", lat0, lon0);

    // Send an RMB with a destination to the NE of the centre (50.5N, 0.5W).
    std::string rmb = Frame('$',
        "GPRMB,A,0.00,L,ORIG,DEST,5030.00,N,00030.00,W,30.0,45.0,30.0,V");
    printf("     -> sending: %s", rmb.c_str());
    send(c, rmb.c_str(), (int)rmb.size(), 0);

    // Drain the socket so the connection stays healthy while we wait.
    std::thread drain([&] {
        char buf[4096];
        auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(7)) {
            recv(c, buf, sizeof(buf), 0);
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(2));
    check(sim.AutopilotEngaged(), "autopilot engaged after RMB received");

    std::this_thread::sleep_for(std::chrono::seconds(4));
    double lat1, lon1, cog1;
    { std::lock_guard<std::mutex> lk(g_m); lat1 = g_lat; lon1 = g_lon; cog1 = g_cog; }
    printf("     steered pos: %.5f, %.5f  cog=%.1f\n", lat1, lon1, cog1);

    // Heading NE toward the waypoint: latitude and longitude both increase, and
    // COG is in the NE quadrant.
    check(lat1 > lat0, "latitude increasing (moving north toward waypoint)");
    check(lon1 > lon0, "longitude increasing (moving east toward waypoint)");
    check(cog1 > 5.0 && cog1 < 85.0, "COG points NE toward waypoint");

    drain.join();
    sim.Stop();
    net.Stop();
    closesocket(c);
    WSACleanup();

    printf("\n%s (%d failures)\n", fail ? "FAILURES PRESENT" : "ALL PASSED", fail);
    return fail ? 1 : 0;
}
