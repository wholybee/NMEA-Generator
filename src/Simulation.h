// Simulation.h - Drives ownship + AIS target motion and emits NMEA sentences.
#pragma once

#include "Geo.h"
#include "Ais.h"

#include <string>
#include <array>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

namespace nmea {

constexpr int kNumTargets = 4;

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
    Shape shape = Shape::Circle;
    double offsetX = 0.0;      // NM east offset of path centre
    double offsetY = 0.0;      // NM north offset of path centre
    double speed = 8.0;        // knots
};

// Full simulation configuration edited through the dialog.
struct SimConfig {
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

    void Start();
    void Stop();
    bool Running() const { return running_.load(); }

private:
    struct Entity {
        double t = 0.0;          // path parameter (radians)
        double prevCog = 0.0;
        bool hasPrev = false;
    };

    void Run();
    void StepOwnship(double dt, const std::tm& utc, bool emit);
    void StepTarget(int i, double dt, const std::tm& utc, bool emitDynamic, bool emitStatic);

    Sink sink_;
    std::atomic<bool> running_{false};
    std::thread thread_;

    std::mutex cfgMutex_;
    SimConfig cfg_;

    Entity ownship_;
    std::array<Entity, kNumTargets> targetEntities_;
    std::array<AisStatic, kNumTargets> targetStatic_;
    int aisChannelToggle_ = 0;
    int aisSeqId_ = 0;
};

// Build a pseudo-random but stable static identity for target index 'i'.
AisStatic MakeStaticIdentity(int i, bool classA);

} // namespace nmea
