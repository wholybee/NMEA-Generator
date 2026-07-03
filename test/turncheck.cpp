// turncheck.cpp - Verify the ownship heading cannot change faster than the
// 6 deg/s turn-rate limit, by forcing a large course change via autopilot.
#include "../src/Simulation.h"
#include "../src/Network.h"
#include "../src/Nmea.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>

#pragma comment(lib, "Ws2_32.lib")

using namespace nmea;

static std::mutex g_m;
struct Sample { double t; double cog; };
static std::vector<Sample> g_samples;
static std::chrono::steady_clock::time_point g_start;

static double AngleDiff(double a, double b) {
    return std::fmod(a - b + 540.0, 360.0) - 180.0;
}

static void CaptureRmc(const std::string& line) {
    if (SentenceFormatter(line) != "RMC") return;
    auto f = SplitFields(line);
    if (f.size() < 9 || f[8].empty()) return;
    double cog;
    try { cog = std::stod(f[8]); } catch (...) { return; }
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - g_start).count();
    std::lock_guard<std::mutex> lk(g_m);
    g_samples.push_back({ t, cog });
}

int main() {
    WSADATA w; WSAStartup(MAKEWORD(2, 2), &w);
    g_start = std::chrono::steady_clock::now();

    NetworkServer net;
    std::string err;
    if (!net.Start(10997, 10997, err)) { printf("server failed: %s\n", err.c_str()); return 1; }

    Simulation sim([&](const std::string& line) { net.Send(line); CaptureRmc(line); });
    net.SetReceiveSink([&](const std::string& line) { sim.OnIncomingSentence(line); });

    SimConfig cfg;
    cfg.ownship.centreLat = 50.0;
    cfg.ownship.centreLon = -1.0;
    cfg.ownship.speed = 40.0;
    cfg.ownship.shape = Shape::Circle;
    sim.SetConfig(cfg);
    sim.Start();

    SOCKET c = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(10997);
    InetPtonA(AF_INET, "127.0.0.1", &a.sin_addr);
    int conn = -1;
    for (int i = 0; i < 20 && conn != 0; ++i) {
        conn = connect(c, (sockaddr*)&a, sizeof(a));
        if (conn != 0) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (conn != 0) { printf("connect failed\n"); return 1; }

    std::thread drain([&] {
        char buf[4096];
        auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(16)) {
            recv(c, buf, sizeof(buf), 0);
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Destination far to the SW forces a large (~180 deg) course change.
    std::string rmb = Frame('$',
        "GPRMB,A,0.00,L,ORIG,DEST,4900.00,N,00200.00,W,60.0,225.0,40.0,V");
    printf("-> %s", rmb.c_str());
    send(c, rmb.c_str(), (int)rmb.size(), 0);

    std::this_thread::sleep_for(std::chrono::seconds(13));
    drain.join();
    sim.Stop();
    net.Stop();
    closesocket(c);
    WSACleanup();

    int fail = 0;
    auto check = [&](bool cond, const char* msg) {
        printf("%s: %s\n", cond ? "ok  " : "FAIL", msg);
        if (!cond) ++fail;
    };

    std::vector<Sample> s;
    { std::lock_guard<std::mutex> lk(g_m); s = g_samples; }
    printf("collected %zu COG samples\n", s.size());

    double maxRate = 0.0, totalChange = 0.0;
    for (size_t i = 1; i < s.size(); ++i) {
        double dt = s[i].t - s[i - 1].t;
        if (dt < 0.2) continue;
        double rate = std::fabs(AngleDiff(s[i].cog, s[i - 1].cog)) / dt;
        if (rate > maxRate) maxRate = rate;
        totalChange += std::fabs(AngleDiff(s[i].cog, s[i - 1].cog));
    }
    printf("max turn rate = %.2f deg/s, total heading change = %.0f deg\n", maxRate, totalChange);

    // Allow a little slack for the 1 Hz sampling vs the 4 Hz integration step.
    check(maxRate <= 7.0, "turn rate never exceeds ~6 deg/s");
    check(totalChange > 60.0, "a large course change actually occurred");

    printf("\n%s (%d failures)\n", fail ? "FAILURES PRESENT" : "ALL PASSED", fail);
    return fail ? 1 : 0;
}
