// Network.h - TCP server + UDP broadcast transport for NMEA sentences.
#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>

namespace nmea {

// Serves NMEA text to any number of connected TCP clients and simultaneously
// broadcasts each line via UDP. Also listens for inbound sentences on the same
// TCP connections and UDP port, delivering each complete line to a callback.
// Thread-safe Send().
class NetworkServer {
public:
    // Callback invoked (on the IO thread) for each complete inbound line, with
    // CR/LF stripped.
    using ReceiveSink = std::function<void(const std::string&)>;

    NetworkServer();
    ~NetworkServer();

    // Install the inbound-sentence callback. Call before Start().
    void SetReceiveSink(ReceiveSink sink) { recvSink_ = std::move(sink); }

    // Start listening on tcpPort and broadcasting on udpPort. Returns false and
    // fills 'error' if a socket could not be created/bound.
    bool Start(unsigned short tcpPort, unsigned short udpPort, std::string& error);
    void Stop();
    bool Running() const { return running_.load(); }

    // Send one line (CRLF already included) to all TCP clients and via UDP.
    void Send(const std::string& line);

    int ClientCount() const;

private:
    struct Client {
        uintptr_t sock;
        std::string rx; // partial inbound line buffer
    };

    void IoLoop();
    void DeliverLines(std::string& buffer, const char* data, int len);

    std::atomic<bool> running_{false};
    uintptr_t listenSock_ = ~uintptr_t(0);
    uintptr_t udpSock_ = ~uintptr_t(0);
    unsigned short udpPort_ = 0;

    ReceiveSink recvSink_;

    std::thread ioThread_;
    mutable std::mutex clientsMutex_;
    std::vector<Client> clients_;
};

} // namespace nmea
