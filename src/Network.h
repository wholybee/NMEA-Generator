// Network.h - TCP server + UDP broadcast transport for NMEA sentences.
#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>

namespace nmea {

// Serves NMEA text to any number of connected TCP clients and simultaneously
// broadcasts each line via UDP. Thread-safe Send().
class NetworkServer {
public:
    NetworkServer();
    ~NetworkServer();

    // Start listening on tcpPort and broadcasting on udpPort. Returns false and
    // fills 'error' if a socket could not be created/bound.
    bool Start(unsigned short tcpPort, unsigned short udpPort, std::string& error);
    void Stop();
    bool Running() const { return running_.load(); }

    // Send one line (CRLF already included) to all TCP clients and via UDP.
    void Send(const std::string& line);

    int ClientCount() const;

private:
    void AcceptLoop();

    std::atomic<bool> running_{false};
    uintptr_t listenSock_ = ~uintptr_t(0);
    uintptr_t udpSock_ = ~uintptr_t(0);
    unsigned short udpPort_ = 0;

    std::thread acceptThread_;
    mutable std::mutex clientsMutex_;
    std::vector<uintptr_t> clients_;
};

} // namespace nmea
