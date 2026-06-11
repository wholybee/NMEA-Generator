// Simulation.cpp - Drives ownship + AIS target motion and emits NMEA sentences.
#include "Simulation.h"
#include "Nmea.h"

#include <chrono>
#include <cmath>
#include <random>
#include <ctime>

namespace nmea {

namespace {

std::tm UtcNow() {
    std::time_t t = std::time(nullptr);
    std::tm out{};
#if defined(_WIN32)
    gmtime_s(&out, &t);
#else
    gmtime_r(&t, &out);
#endif
    return out;
}

// Advance the path parameter so the entity moves ~speed knots over dt seconds.
double Advance(Shape shape, double t, double w, double h, double speedKnots, double dt) {
    const double distNm = speedKnots * (dt / 3600.0);
    double tangent = PathSpeed(shape, t, w, h); // NM per radian
    if (tangent < 1e-6) tangent = 1e-6;
    return t + distNm / tangent;
}

// Smallest signed difference a-b mapped to [-180,180].
double AngleDiff(double a, double b) {
    double d = std::fmod(a - b + 540.0, 360.0) - 180.0;
    return d;
}

} // namespace

AisStatic MakeStaticIdentity(int i, bool classA) {
    // Deterministic per-index generator so identities are stable across runs.
    std::mt19937 rng(0xA15u + i * 101u);
    auto rnd = [&](int lo, int hi) { return lo + static_cast<int>(rng() % (hi - lo + 1)); };

    static const char* kNames[] = {
        "SEA EXPLORER", "NORTHERN STAR", "BLUE HORIZON", "CORAL QUEEN",
        "IRON DUKE", "PACIFIC DAWN", "STORM PETREL", "GOLDEN ARROW"
    };
    static const char* kPrefix[] = { "MV", "FV", "SV", "MY" };

    AisStatic s;
    s.classA = classA;
    s.mmsi = 235000000u + i * 111111u + rnd(0, 9999);
    s.imo = classA ? (9000000u + rnd(0, 999999)) : 0u;
    s.name = kNames[i % 8];
    {
        char cs[8];
        std::snprintf(cs, sizeof(cs), "%c%c%d%d%d%d",
                      'A' + rnd(0, 25), 'A' + rnd(0, 25),
                      rnd(0, 9), rnd(0, 9), rnd(0, 9), rnd(0, 9));
        s.callsign = cs;
    }
    // Ship & cargo type: 70 = cargo, 30 = fishing, 36 = sailing, 37 = pleasure.
    static const int kTypes[] = { 70, 30, 36, 37, 60, 80 };
    s.shipType = static_cast<uint8_t>(kTypes[rnd(0, 5)]);
    s.dimBow = static_cast<uint16_t>(rnd(20, 120));
    s.dimStern = static_cast<uint16_t>(rnd(10, 40));
    s.dimPort = static_cast<uint8_t>(rnd(5, 20));
    s.dimStarboard = static_cast<uint8_t>(rnd(5, 20));
    (void)kPrefix;
    return s;
}

Simulation::Simulation(Sink sink) : sink_(std::move(sink)) {
    for (int i = 0; i < kNumTargets; ++i) {
        cfg_.targets[i].offsetX = (i - 1.5) * 1.5;
        cfg_.targets[i].offsetY = (i % 2 == 0) ? 1.0 : -1.0;
        cfg_.targets[i].classA = (i % 2 == 0);
        cfg_.targets[i].shape = static_cast<Shape>(i % 3);
        targetStatic_[i] = MakeStaticIdentity(i, cfg_.targets[i].classA);
        targetEntities_[i].t = i * 1.2;
    }
}

Simulation::~Simulation() { Stop(); }

void Simulation::SetConfig(const SimConfig& cfg) {
    std::lock_guard<std::mutex> lock(cfgMutex_);
    cfg_ = cfg;
    for (int i = 0; i < kNumTargets; ++i) {
        if (targetStatic_[i].classA != cfg_.targets[i].classA) {
            targetStatic_[i] = MakeStaticIdentity(i, cfg_.targets[i].classA);
        }
    }
}

void Simulation::Start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread(&Simulation::Run, this);
}

void Simulation::Stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
}

void Simulation::Run() {
    using clock = std::chrono::steady_clock;
    auto last = clock::now();
    double ownAccum = 1.0;     // seconds since last ownship emit (1s)
    double dynAccum = 6.0;     // seconds since last dynamic AIS emit (6s)
    double staticAccum = 60.0; // force static emit on first iteration

    while (running_.load()) {
        auto now = clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        std::tm utc = UtcNow();

        ownAccum += dt;
        dynAccum += dt;
        staticAccum += dt;
        const bool emitOwn = ownAccum >= 1.0;
        const bool emitDynamic = dynAccum >= 6.0;
        const bool emitStatic = staticAccum >= 60.0;

        // Motion is integrated every tick; sentences are emitted on cadence.
        StepOwnship(dt, utc, emitOwn);
        for (int i = 0; i < kNumTargets; ++i) {
            StepTarget(i, dt, utc, emitDynamic, emitStatic);
        }

        if (emitOwn) ownAccum = 0.0;
        if (emitDynamic) dynAccum = 0.0;
        if (emitStatic) staticAccum = 0.0;

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
}

void Simulation::StepOwnship(double dt, const std::tm& utc, bool emit) {
    OwnshipConfig oc;
    {
        std::lock_guard<std::mutex> lock(cfgMutex_);
        oc = cfg_.ownship;
    }

    Vec2 before = PathPoint(oc.shape, ownship_.t, oc.widthNm, oc.heightNm);
    ownship_.t = Advance(oc.shape, ownship_.t, oc.widthNm, oc.heightNm, oc.speed, dt);
    Vec2 after = PathPoint(oc.shape, ownship_.t, oc.widthNm, oc.heightNm);

    OwnshipState st;
    OffsetToLatLon(oc.centreLat, oc.centreLon, after.x, after.y, st.latitude, st.longitude);

    const double dx = after.x - before.x;
    const double dy = after.y - before.y;
    st.cog = (std::fabs(dx) + std::fabs(dy) > 1e-9) ? BearingDeg(dx, dy) : ownship_.prevCog;
    st.sog = oc.speed;
    st.heading = st.cog;
    st.speedThroughWater = oc.speed;

    // Synthesised wind: a steady true wind from the NW, apparent derived from it.
    st.trueWindDir = 315.0;
    st.trueWindSpeed = 12.0;
    {
        // Vector subtract vessel motion from true wind to get apparent wind.
        const double twRad = (st.trueWindDir + 180.0) * M_PI / 180.0; // blowing-to
        double twx = st.trueWindSpeed * std::sin(twRad);
        double twy = st.trueWindSpeed * std::cos(twRad);
        const double hdgRad = st.heading * M_PI / 180.0;
        double vx = st.sog * std::sin(hdgRad);
        double vy = st.sog * std::cos(hdgRad);
        double ax = twx - vx;
        double ay = twy - vy;
        st.appWindSpeed = std::sqrt(ax * ax + ay * ay);
        double appDirTo = BearingDeg(ax, ay);
        double appFrom = std::fmod(appDirTo + 180.0, 360.0);
        st.appWindAngle = std::fmod(appFrom - st.heading + 360.0, 360.0);
    }

    ownship_.prevCog = st.cog;
    ownship_.hasPrev = true;

    if (emit) {
        for (const std::string& line : BuildOwnshipSentences(st, utc)) {
            sink_(line);
        }
    }
}

void Simulation::StepTarget(int i, double dt, const std::tm& utc, bool emitDynamic, bool emitStatic) {
    TargetConfig tc;
    OwnshipConfig oc;
    {
        std::lock_guard<std::mutex> lock(cfgMutex_);
        tc = cfg_.targets[i];
        oc = cfg_.ownship;
    }
    if (!tc.enabled) return;

    Entity& e = targetEntities_[i];

    Vec2 before = PathPoint(tc.shape, e.t, oc.widthNm, oc.heightNm);
    e.t = Advance(tc.shape, e.t, oc.widthNm, oc.heightNm, tc.speed, dt);
    Vec2 after = PathPoint(tc.shape, e.t, oc.widthNm, oc.heightNm);

    const double xNm = after.x + tc.offsetX;
    const double yNm = after.y + tc.offsetY;

    AisDynamic d;
    d.mmsi = targetStatic_[i].mmsi;
    d.classA = tc.classA;
    OffsetToLatLon(oc.centreLat, oc.centreLon, xNm, yNm, d.latitude, d.longitude);

    const double dx = after.x - before.x;
    const double dy = after.y - before.y;
    double cog = (std::fabs(dx) + std::fabs(dy) > 1e-9) ? BearingDeg(dx, dy) : e.prevCog;
    d.cog = cog;
    d.sog = tc.speed;
    d.heading = cog;
    d.timestamp = utc.tm_sec;

    if (e.hasPrev && dt > 1e-6) {
        d.rot = AngleDiff(cog, e.prevCog) / (dt / 60.0); // deg per minute
        if (d.rot > 360.0) d.rot = 360.0;
        if (d.rot < -360.0) d.rot = -360.0;
    } else {
        d.rot = 0.0;
    }
    e.prevCog = cog;
    e.hasPrev = true;

    if (!emitDynamic && !emitStatic) return;

    // Channel alternates A/B between successive AIS sentences.
    const char channel = (aisChannelToggle_++ & 1) ? 'B' : 'A';

    if (emitDynamic) sink_(EncodePositionReport(d, channel));

    if (emitStatic) {
        for (const std::string& s : EncodeStaticReport(targetStatic_[i], channel, aisSeqId_)) {
            sink_(s);
        }
    }
}

} // namespace nmea
