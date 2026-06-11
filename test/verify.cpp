// verify.cpp - Standalone sanity checks for the NMEA/AIS encoders.
// Compile (from a VS x64 dev prompt):
//   cl /EHsc /std:c++17 /I..\src verify.cpp ..\src\Ais.cpp ..\src\Nmea.cpp
#include "../src/Ais.h"
#include "../src/Nmea.h"

#include <cstdio>
#include <string>
#include <vector>
#include <cstdint>
#include <cassert>
#include <cmath>

using namespace nmea;

static int g_fail = 0;
#define CHECK(cond, msg) do { if(!(cond)){ printf("FAIL: %s\n", msg); ++g_fail; } else { printf("ok  : %s\n", msg); } } while(0)

// Decode the AIS 6-bit armour back into a bit string.
static std::string Dearmor(const std::string& payload) {
    std::string bits;
    for (char c : payload) {
        int v = (unsigned char)c - 48;
        if (v > 40) v -= 8;
        for (int i = 5; i >= 0; --i) bits.push_back(((v >> i) & 1) ? '1' : '0');
    }
    return bits;
}

static uint64_t GetU(const std::string& bits, int start, int len) {
    uint64_t v = 0;
    for (int i = 0; i < len; ++i) v = (v << 1) | (bits[start + i] - '0');
    return v;
}

static int64_t GetI(const std::string& bits, int start, int len) {
    uint64_t v = GetU(bits, start, len);
    if (v & (1ULL << (len - 1))) v |= ~((1ULL << len) - 1); // sign extend
    return (int64_t)v;
}

// Extract the 6th comma-separated field (the payload) from an !AIVDM sentence.
static std::string PayloadOf(const std::string& sentence) {
    int comma = 0; size_t i = 0;
    for (; i < sentence.size() && comma < 5; ++i) if (sentence[i] == ',') ++comma;
    size_t end = sentence.find(',', i);
    return sentence.substr(i, end - i);
}

static bool ChecksumOk(const std::string& s) {
    size_t star = s.find('*');
    if (star == std::string::npos || s[0] != '!' && s[0] != '$') return false;
    unsigned char cs = 0;
    for (size_t i = 1; i < star; ++i) cs ^= (unsigned char)s[i];
    char buf[4]; std::snprintf(buf, sizeof(buf), "%02X", cs);
    return s.compare(star + 1, 2, buf) == 0;
}

int main() {
    // --- Type 1 position report round-trip ---
    AisDynamic d;
    d.mmsi = 235123456;
    d.latitude = 50.8;
    d.longitude = -1.1;
    d.sog = 12.3;
    d.cog = 90.0;
    d.heading = 88;
    d.timestamp = 42;
    d.classA = true;
    std::string s1 = EncodePositionReport(d, 'A');
    printf("Type1: %s", s1.c_str());
    CHECK(ChecksumOk(s1), "type1 checksum");
    {
        std::string bits = Dearmor(PayloadOf(s1));
        CHECK(GetU(bits, 0, 6) == 1, "type1 message id == 1");
        CHECK(GetU(bits, 8, 30) == 235123456u, "type1 mmsi round-trip");
        double lon = GetI(bits, 61, 28) / 600000.0;
        double lat = GetI(bits, 89, 27) / 600000.0;
        CHECK(std::fabs(lon - (-1.1)) < 1e-4, "type1 longitude round-trip");
        CHECK(std::fabs(lat - 50.8) < 1e-4, "type1 latitude round-trip");
        CHECK(GetU(bits, 50, 10) == 123, "type1 sog (12.3kn -> 123)");
    }

    // --- Type 18 (Class B) ---
    d.classA = false;
    std::string s18 = EncodePositionReport(d, 'B');
    printf("Type18: %s", s18.c_str());
    CHECK(ChecksumOk(s18), "type18 checksum");
    {
        std::string bits = Dearmor(PayloadOf(s18));
        CHECK(GetU(bits, 0, 6) == 18, "type18 message id == 18");
        CHECK(GetU(bits, 8, 30) == 235123456u, "type18 mmsi round-trip");
    }

    // --- Type 5 static (multi-part) ---
    AisStatic st;
    st.mmsi = 235111222; st.imo = 9123456; st.name = "SEA EXPLORER";
    st.callsign = "ABCD12"; st.shipType = 70;
    st.dimBow = 80; st.dimStern = 20; st.dimPort = 10; st.dimStarboard = 10;
    st.classA = true;
    int seq = 1;
    auto v5 = EncodeStaticReport(st, 'A', seq);
    printf("Type5 parts: %zu\n", v5.size());
    CHECK(v5.size() == 2, "type5 spans 2 sentences");
    for (auto& s : v5) { printf("  %s", s.c_str()); CHECK(ChecksumOk(s), "type5 fragment checksum"); }
    {
        // Reassemble payload from both fragments and decode MMSI.
        std::string bits = Dearmor(PayloadOf(v5[0])) + Dearmor(PayloadOf(v5[1]));
        CHECK(GetU(bits, 0, 6) == 5, "type5 message id == 5");
        CHECK(GetU(bits, 8, 30) == st.mmsi, "type5 mmsi round-trip");
        CHECK(GetU(bits, 40, 30) == st.imo, "type5 imo round-trip");
    }

    // --- Type 24 (Class B static, parts A & B) ---
    AisStatic stb = st;
    stb.mmsi = 235333444; stb.imo = 0; stb.name = "BLUE HORIZON";
    stb.callsign = "WXYZ99"; stb.classA = false;
    int seq2 = 1;
    auto v24 = EncodeStaticReport(stb, 'B', seq2);
    printf("Type24 parts: %zu\n", v24.size());
    CHECK(v24.size() == 2, "type24 has Part A + Part B");
    {
        std::string a = Dearmor(PayloadOf(v24[0]));
        std::string b = Dearmor(PayloadOf(v24[1]));
        CHECK(GetU(a, 0, 6) == 24 && GetU(a, 38, 2) == 0, "type24 part A");
        CHECK(GetU(b, 0, 6) == 24 && GetU(b, 38, 2) == 1, "type24 part B");
    }

    // --- Ownship sentences checksums ---
    OwnshipState os;
    os.latitude = 50.8; os.longitude = -1.1; os.cog = 123.4; os.sog = 9.9;
    os.heading = 120; os.speedThroughWater = 9.5;
    os.appWindAngle = 45; os.appWindSpeed = 14; os.trueWindDir = 315; os.trueWindSpeed = 12;
    std::tm utc{}; utc.tm_hour = 12; utc.tm_min = 34; utc.tm_sec = 56;
    utc.tm_mday = 10; utc.tm_mon = 5; utc.tm_year = 126;
    auto own = BuildOwnshipSentences(os, utc);
    printf("Ownship sentences: %zu\n", own.size());
    for (auto& s : own) { printf("  %s", s.c_str()); CHECK(ChecksumOk(s), "ownship checksum"); }

    printf("\n%s (%d failures)\n", g_fail ? "FAILURES PRESENT" : "ALL PASSED", g_fail);
    return g_fail ? 1 : 0;
}
