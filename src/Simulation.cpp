// Simulation.cpp - Drives ownship + AIS target motion and emits NMEA sentences.
#include "Simulation.h"
#include "Nmea.h"
#include "N2k.h"
#include "ApInput.h"

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

// Maximum rate at which the ownship heading may change, modelling the time a
// real vessel needs to come around to a new course.
constexpr double kMaxTurnRateDegPerSec = 6.0;

// Rotate 'current' heading toward 'target' by at most maxStep degrees, taking
// the shortest way round. Result normalised to [0,360).
double SlewHeading(double current, double target, double maxStep) {
    double diff = AngleDiff(target, current);
    if (diff > maxStep) diff = maxStep;
    else if (diff < -maxStep) diff = -maxStep;
    double r = std::fmod(current + diff, 360.0);
    if (r < 0.0) r += 360.0;
    return r;
}

} // namespace

AisStatic MakeStaticIdentity(int i, AisTargetKind kind) {
    // Deterministic per-index generator so identities are stable across runs.
    std::mt19937 rng(0xA15u + i * 101u);
    auto rnd = [&](int lo, int hi) { return lo + static_cast<int>(rng() % (hi - lo + 1)); };

    static const char* kNames[] = {
        "SEA EXPLORER", "NORTHERN STAR", "BLUE HORIZON", "CORAL QUEEN",
        "IRON DUKE", "PACIFIC DAWN", "STORM PETREL", "GOLDEN ARROW"
    };
    AisStatic s;
    s.kind = kind;
    s.classA = (kind == AisTargetKind::ClassA);
    if (kind == AisTargetKind::SarFixedWing) {
        s.mmsi = 111366100u + static_cast<uint32_t>((i * 37 + rnd(0, 399)) % 400);
        s.name = "SAR FIXED WING";
    } else if (kind == AisTargetKind::SarHelicopter) {
        s.mmsi = 111366500u + static_cast<uint32_t>((i * 53 + rnd(0, 499)) % 500);
        s.name = "SAR HELICOPTER";
    } else {
        s.mmsi = 235000000u + i * 111111u + rnd(0, 9999);
        s.name = kNames[i % 8];
    }
    s.imo = s.classA ? (9000000u + rnd(0, 999999)) : 0u;
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
    return s;
}

Simulation::Simulation(Sink sink) : sink_(std::move(sink)) {
    for (int i = 0; i < kNumTargets; ++i) {
        cfg_.targets[i].offsetX = (i - 1.5) * 1.5;
        cfg_.targets[i].offsetY = (i % 2 == 0) ? 1.0 : -1.0;
        cfg_.targets[i].kind = (i % 2 == 0) ? AisTargetKind::ClassA : AisTargetKind::ClassB;
        cfg_.targets[i].classA = (cfg_.targets[i].kind == AisTargetKind::ClassA);
        cfg_.targets[i].shape = static_cast<Shape>(i % 3);
        targetStatic_[i] = MakeStaticIdentity(i, cfg_.targets[i].kind);
        targetEntities_[i].t = i * 1.2;
    }
}

Simulation::~Simulation() { Stop(); }

namespace {
constexpr double kAutopilotTimeoutSec = 15.0; // revert to pattern if AP goes quiet
constexpr double kArrivalNm = 0.03;            // hold heading within this of dest
} // namespace

void Simulation::OnIncomingSentence(const std::string& line) {
    ApInput in;
    if (!ParseAutopilotInput(line, in) &&
        !ParseN2kAutopilotInput(line, in)) {
        return; // not an APB/RMB/XTE sentence or autopilot PGN
    }
    if (!in.ok()) return; // never steer from malformed / invalid autopilot data

    std::lock_guard<std::mutex> lock(apMutex_);
    if (in.hasXte) ap_.xteNm = in.xteNm;
    if (in.xteDir) ap_.steer = in.xteDir;
    if (in.hasDest) { ap_.hasDest = true; ap_.destLat = in.destLat; ap_.destLon = in.destLon; }
    if (in.hasSteerBearing) { ap_.hasBearing = true; ap_.bearing = in.steerBearing; }
    ap_.lastUpdate = std::chrono::steady_clock::now();
    ap_.everReceived = true;
}

void Simulation::SetConfig(const SimConfig& cfg) {
    std::lock_guard<std::mutex> lock(cfgMutex_);
    cfg_ = cfg;
    for (int i = 0; i < kNumTargets; ++i) {
        cfg_.targets[i].classA = (cfg_.targets[i].kind == AisTargetKind::ClassA);
        if (targetStatic_[i].kind != cfg_.targets[i].kind) {
            targetStatic_[i] = MakeStaticIdentity(i, cfg_.targets[i].kind);
        }
    }
}

void Simulation::TriggerMob() {
    OwnshipState own;
    bool haveOwn = false;
    {
        std::lock_guard<std::mutex> lock(ownStateMutex_);
        own = lastOwnship_;
        haveOwn = hasLastOwnship_;
    }

    if (!haveOwn) {
        OwnshipConfig oc;
        {
            std::lock_guard<std::mutex> lock(cfgMutex_);
            oc = cfg_.ownship;
        }
        own.latitude = oc.centreLat;
        own.longitude = oc.centreLon;
    }

    std::lock_guard<std::mutex> lock(mobMutex_);
    mob_.active = true;
    mob_.lat = own.latitude;
    mob_.lon = own.longitude;
    mob_.until = std::chrono::steady_clock::now() + std::chrono::minutes(2);
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
    double mobAccum = 6.0;     // force MOB emit immediately when active

    while (running_.load()) {
        auto now = clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        std::tm utc = UtcNow();

        ownAccum += dt;
        dynAccum += dt;
        staticAccum += dt;
        mobAccum += dt;
        const bool emitOwn = ownAccum >= 1.0;
        const bool emitDynamic = dynAccum >= 6.0;
        const bool emitStatic = staticAccum >= 60.0;
        const bool emitMob = mobAccum >= 6.0;

        // Motion is integrated every tick; sentences are emitted on cadence.
        StepOwnship(dt, utc, emitOwn);
        for (int i = 0; i < kNumTargets; ++i) {
            StepTarget(i, dt, utc, emitDynamic, emitStatic);
        }
        StepMob(utc, emitMob);

        if (emitOwn) ownAccum = 0.0;
        if (emitDynamic) dynAccum = 0.0;
        if (emitStatic) staticAccum = 0.0;
        if (emitMob) mobAccum = 0.0;

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
}

void Simulation::StepOwnship(double dt, const std::tm& utc, bool emit) {
    OwnshipConfig oc;
    ProtocolMode protocol;
    {
        std::lock_guard<std::mutex> lock(cfgMutex_);
        oc = cfg_.ownship;
        protocol = cfg_.protocol;
    }

    // Decide whether autopilot mode is engaged (recent APB/RMB/XTE received).
    Autopilot ap;
    bool engaged = false;
    {
        std::lock_guard<std::mutex> lock(apMutex_);
        ap = ap_;
        if (ap.everReceived) {
            const double age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - ap.lastUpdate).count();
            engaged = age < kAutopilotTimeoutSec;
        }
    }
    apEngaged_.store(engaged);

    OwnshipState st;

    // The heading the helm is calling for this tick; the actual heading slews
    // toward it at no more than kMaxTurnRateDegPerSec.
    double desiredHeading;
    const double maxStep = kMaxTurnRateDegPerSec * dt;

    if (engaged) {
        // --- Autopilot: steer toward the destination / commanded bearing. ---
        if (!apPosInit_) {
            // Seed the integrated position from the current pattern position.
            Vec2 p = PathPoint(oc.shape, ownship_.t, oc.widthNm, oc.heightNm);
            OffsetToLatLon(oc.centreLat, oc.centreLon, p.x, p.y, apLat_, apLon_);
            apPosInit_ = true;
        }

        if (ap.hasDest) {
            double brg, dist;
            BearingDistanceLatLon(apLat_, apLon_, ap.destLat, ap.destLon, brg, dist);
            desiredHeading = (dist > kArrivalNm) ? brg : ownship_.heading; // hold on arrival
        } else if (ap.hasBearing) {
            desiredHeading = ap.bearing;
        } else {
            desiredHeading = ownship_.heading; // XTE only: hold current heading
        }

        if (!ownship_.headingInit) { ownship_.heading = desiredHeading; ownship_.headingInit = true; }
        else ownship_.heading = SlewHeading(ownship_.heading, desiredHeading, maxStep);

        // Steam along the (rate-limited) actual heading, so the boat curves into
        // its turn rather than pivoting on the spot.
        const double distNm = oc.speed * (dt / 3600.0);
        MoveLatLon(apLat_, apLon_, ownship_.heading, distNm, apLat_, apLon_);

        st.latitude = apLat_;
        st.longitude = apLon_;
        st.sog = oc.speed;
        st.speedThroughWater = oc.speed;
    } else {
        // --- Predefined pattern. ---
        apPosInit_ = false; // re-seed next time autopilot engages

        Vec2 before = PathPoint(oc.shape, ownship_.t, oc.widthNm, oc.heightNm);
        ownship_.t = Advance(oc.shape, ownship_.t, oc.widthNm, oc.heightNm, oc.speed, dt);
        Vec2 after = PathPoint(oc.shape, ownship_.t, oc.widthNm, oc.heightNm);

        OffsetToLatLon(oc.centreLat, oc.centreLon, after.x, after.y, st.latitude, st.longitude);

        const double dx = after.x - before.x;
        const double dy = after.y - before.y;
        desiredHeading = (std::fabs(dx) + std::fabs(dy) > 1e-9) ? BearingDeg(dx, dy) : ownship_.heading;

        if (!ownship_.headingInit) { ownship_.heading = desiredHeading; ownship_.headingInit = true; }
        else ownship_.heading = SlewHeading(ownship_.heading, desiredHeading, maxStep);

        st.sog = oc.speed;
        st.speedThroughWater = oc.speed;
    }

    // Heading and course both follow the rate-limited heading.
    st.heading = ownship_.heading;
    st.cog = ownship_.heading;

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
    {
        std::lock_guard<std::mutex> lock(ownStateMutex_);
        lastOwnship_ = st;
        hasLastOwnship_ = true;
    }

    if (emit) {
        const std::vector<std::string> lines =
            (protocol == ProtocolMode::Nmea2000)
                ? BuildN2kOwnshipMessages(st, utc)
                : BuildOwnshipSentences(st, utc);
        for (const std::string& line : lines) {
            sink_(line);
        }
    }
}

void Simulation::StepMob(const std::tm& utc, bool emit) {
    if (!emit) return;

    ProtocolMode protocol;
    MobState mob;
    {
        std::lock_guard<std::mutex> cfgLock(cfgMutex_);
        protocol = cfg_.protocol;
    }
    {
        std::lock_guard<std::mutex> lock(mobMutex_);
        if (!mob_.active) return;
        if (std::chrono::steady_clock::now() > mob_.until) {
            mob_.active = false;
            return;
        }
        mob = mob_;
    }

    constexpr uint32_t kMobMmsi = 972366001u;
    AisDynamic d;
    d.mmsi = kMobMmsi;
    d.classA = true;
    d.kind = AisTargetKind::ClassA;
    d.latitude = mob.lat;
    d.longitude = mob.lon;
    d.sog = 0.0;
    d.cog = 0.0;
    d.heading = 511.0;
    d.rot = 0.0;
    d.timestamp = utc.tm_sec;

    const char channel = (aisChannelToggle_++ & 1) ? 'B' : 'A';
    if (protocol == ProtocolMode::Nmea2000) {
        for (const std::string& s : BuildN2kAisDynamicMessages(d)) sink_(s);
        sink_(BuildN2kSafetyBroadcast(kMobMmsi, "MOB ACTIVE"));
    } else {
        sink_(EncodePositionReport(d, channel));
        sink_(EncodeSafetyBroadcast(kMobMmsi, "MOB ACTIVE", channel));
    }
}

void Simulation::StepTarget(int i, double dt, const std::tm& utc, bool emitDynamic, bool emitStatic) {
    TargetConfig tc;
    OwnshipConfig oc;
    ProtocolMode protocol;
    {
        std::lock_guard<std::mutex> lock(cfgMutex_);
        tc = cfg_.targets[i];
        oc = cfg_.ownship;
        protocol = cfg_.protocol;
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
    d.kind = tc.kind;
    d.altitudeMeters = IsSarKind(tc.kind)
        ? ((tc.kind == AisTargetKind::SarHelicopter) ? 150 : 1500)
        : 0;
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

    if (emitDynamic) {
        if (protocol == ProtocolMode::Nmea2000) {
            for (const std::string& s : BuildN2kAisDynamicMessages(d)) sink_(s);
        } else {
            sink_(EncodePositionReport(d, channel));
        }
    }

    if (emitStatic && !IsSarKind(tc.kind)) {
        const std::vector<std::string> lines =
            (protocol == ProtocolMode::Nmea2000)
                ? BuildN2kAisStaticMessages(targetStatic_[i])
                : EncodeStaticReport(targetStatic_[i], channel, aisSeqId_);
        for (const std::string& s : lines) {
            sink_(s);
        }
    }
}

} // namespace nmea
