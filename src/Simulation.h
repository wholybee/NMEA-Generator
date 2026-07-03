// Simulation.h - Drives ownship + AIS target motion and emits NMEA sentences.
#pragma once

#include "Geo.h"
#include "Ais.h"
#include "Nmea.h"

#include <string>
#include <array>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

namespace nmea {

constexpr int kNumTargets = 4;

enum class ProtocolMode {
    Nmea0183 = 0,
    Nmea2000 = 1
};

// User-editable configuration for the ownship path.
struct OwnshipConfig {
    double centreLat = 50.0;   // simulation centre latitude
    double centreLon = -1.0;   // simulation centre longitude
    double widthNm = 5.0;      // E-W extent
    double heightNm = 5.0;     // N-S extent
    Shape shape = Shape::Circle;
    double speed = 10.0;       // knots
};

// User-editable configuration for one AIS target.
struct TargetConfig {
    bool enabled = true;
    bool classA = true;
    AisTargetKind kind = AisTargetKind::ClassA;
    Shape shape = Shape::Circle;
    double offsetX = 0.0;      // NM east offset of path centre
    double offsetY = 0.0;      // NM north offset of path centre
    double speed = 8.0;        // knots
};

// Full simulation configuration edited through the dialog.
struct SimConfig {
    ProtocolMode protocol = ProtocolMode::Nmea0183;
    OwnshipConfig ownship;
    std::array<TargetConfig, kNumTargets> targets;
};

// Runs the simulation on a background thread. Each generated sentence is
// delivered through the sink callback (which the app wires to the network
// server and the on-screen log).
class Simulation {
public:
    using Sink = std::function<void(const std::string&)>;

    explicit Simulation(Sink sink);
    ~Simulation();

    // Replace the live configuration (thread-safe).
    void SetConfig(const SimConfig& cfg);

    // Feed an incoming NMEA 0183 sentence or Actisense N2K ASCII line.
    // APB/RMB/XTE or their N2K PGN equivalents engage autopilot mode and steer
    // ownship. Other sentences are ignored. Thread-safe.
    void OnIncomingSentence(const std::string& line);

    // True while the ownship is steering from received autopilot data rather
    // than its predefined pattern.
    bool AutopilotEngaged() const { return apEngaged_.load(); }

    // Start a 2-minute AIS MOB burst at the current ownship position.
    void TriggerMob();

    void Start();
    void Stop();
    bool Running() const { return running_.load(); }

private:
    struct Entity {
        double t = 0.0;          // path parameter (radians)
        double prevCog = 0.0;
        bool hasPrev = false;
        double heading = 0.0;    // rate-limited actual heading (degrees true)
        bool headingInit = false;
    };

    void Run();
    void StepOwnship(double dt, const std::tm& utc, bool emit);
    void StepTarget(int i, double dt, const std::tm& utc, bool emitDynamic, bool emitStatic);
    void StepMob(const std::tm& utc, bool emit);

    Sink sink_;
    std::atomic<bool> running_{false};
    std::thread thread_;

    // Autopilot command most recently decoded from incoming APB/RMB/XTE.
    struct Autopilot {
        bool hasBearing = false;
        double bearing = 0.0;   // degrees true, heading to steer
        bool hasDest = false;
        double destLat = 0.0;
        double destLon = 0.0;
        double xteNm = 0.0;     // cross-track error magnitude
        char steer = 0;         // 'L' or 'R' direction to steer
        std::chrono::steady_clock::time_point lastUpdate;
        bool everReceived = false;
    };

    std::mutex cfgMutex_;
    SimConfig cfg_;

    std::mutex apMutex_;
    Autopilot ap_;
    std::atomic<bool> apEngaged_{false};
    // Integrated ownship position while autopilot is engaged (sim thread only).
    double apLat_ = 0.0;
    double apLon_ = 0.0;
    bool apPosInit_ = false;

    std::mutex ownStateMutex_;
    OwnshipState lastOwnship_;
    bool hasLastOwnship_ = false;

    struct MobState {
        bool active = false;
        double lat = 0.0;
        double lon = 0.0;
        std::chrono::steady_clock::time_point until;
    };
    std::mutex mobMutex_;
    MobState mob_;

    Entity ownship_;
    std::array<Entity, kNumTargets> targetEntities_;
    std::array<AisStatic, kNumTargets> targetStatic_;
    int aisChannelToggle_ = 0;
    int aisSeqId_ = 0;
};

// Build a pseudo-random but stable static identity for target index 'i'.
AisStatic MakeStaticIdentity(int i, AisTargetKind kind);

} // namespace nmea
