// netcheck.cpp - End-to-end smoke test: Simulation -> NetworkServer -> TCP client.
#include "../src/Simulation.h"
#include "../src/Network.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdio>
#include <string>
#include <thread>
#include <chrono>

#pragma comment(lib, "Ws2_32.lib")

using namespace nmea;

int main() {
    WSADATA w; WSAStartup(MAKEWORD(2, 2), &w);

    NetworkServer net;
    std::string err;
    if (!net.Start(10999, 10999, err)) { printf("server start failed: %s\n", err.c_str()); return 1; }

    Simulation sim([&](const std::string& line) { net.Send(line); });

    SimConfig cfg;
    cfg.ownship.speed = 600.0; // fast so we see motion quickly
    sim.SetConfig(cfg);
    sim.Start();

    // Connect a TCP client.
    SOCKET c = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(10999);
    InetPtonA(AF_INET, "127.0.0.1", &a.sin_addr);
    // Retry connect briefly while the accept loop spins up.
    int connected = -1;
    for (int i = 0; i < 20 && connected != 0; ++i) {
        connected = connect(c, (sockaddr*)&a, sizeof(a));
        if (connected != 0) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (connected != 0) { printf("client connect failed\n"); return 1; }
    printf("client connected; collecting ~8s of data...\n");

    int gga = 0, rmc = 0, aivdm = 0, total = 0;
    char buf[4096];
    auto t0 = std::chrono::steady_clock::now();
    std::string acc;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(8)) {
        int n = recv(c, buf, sizeof(buf), 0);
        if (n <= 0) break;
        acc.append(buf, n);
        size_t pos;
        while ((pos = acc.find("\r\n")) != std::string::npos) {
            std::string line = acc.substr(0, pos);
            acc.erase(0, pos + 2);
            ++total;
            if (line.rfind("$GPGGA", 0) == 0) ++gga;
            if (line.rfind("$GPRMC", 0) == 0) ++rmc;
            if (line.rfind("!AIVDM", 0) == 0) ++aivdm;
            if (total <= 14) printf("  %s\n", line.c_str());
        }
    }

    printf("\ntotals: lines=%d  GGA=%d  RMC=%d  AIVDM=%d  clients=%d\n",
           total, gga, rmc, aivdm, net.ClientCount());

    sim.Stop();
    net.Stop();
    closesocket(c);
    WSACleanup();

    bool ok = total > 0 && gga > 0 && rmc > 0 && aivdm > 0;
    printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
