// Network.cpp - TCP server + UDP broadcast transport for NMEA sentences.
#include "Network.h"

#include <winsock2.h>
#include <ws2tcpip.h>

#include <algorithm>

#pragma comment(lib, "Ws2_32.lib")

namespace nmea {

namespace {
struct WsaInit {
    WsaInit() { WSADATA d; WSAStartup(MAKEWORD(2, 2), &d); }
    ~WsaInit() { WSACleanup(); }
};
WsaInit g_wsa;
} // namespace

NetworkServer::NetworkServer() = default;

NetworkServer::~NetworkServer() {
    Stop();
}

bool NetworkServer::Start(unsigned short tcpPort, unsigned short udpPort, std::string& error) {
    if (running_.load()) return true;

    // --- TCP listening socket ---
    SOCKET ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ls == INVALID_SOCKET) { error = "TCP socket() failed"; return false; }

    BOOL yes = TRUE;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&yes), sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(tcpPort);
    if (bind(ls, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        error = "TCP bind() failed on port " + std::to_string(tcpPort);
        closesocket(ls);
        return false;
    }
    if (listen(ls, SOMAXCONN) == SOCKET_ERROR) {
        error = "TCP listen() failed";
        closesocket(ls);
        return false;
    }

    // --- UDP socket: broadcasts outbound and (bound to udpPort) receives inbound ---
    SOCKET us = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (us == INVALID_SOCKET) {
        error = "UDP socket() failed";
        closesocket(ls);
        return false;
    }
    BOOL bcast = TRUE;
    setsockopt(us, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<char*>(&bcast), sizeof(bcast));
    setsockopt(us, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&yes), sizeof(yes));

    sockaddr_in uaddr{};
    uaddr.sin_family = AF_INET;
    uaddr.sin_addr.s_addr = INADDR_ANY;
    uaddr.sin_port = htons(udpPort);
    // Binding lets us receive inbound datagrams on the same UDP port. A failure
    // here (e.g. port already bound) is non-fatal: we can still broadcast.
    bind(us, reinterpret_cast<sockaddr*>(&uaddr), sizeof(uaddr));

    listenSock_ = ls;
    udpSock_ = us;
    udpPort_ = udpPort;
    running_.store(true);

    ioThread_ = std::thread(&NetworkServer::IoLoop, this);
    return true;
}

void NetworkServer::DeliverLines(std::string& buffer, const char* data, int len) {
    buffer.append(data, len);
    size_t pos;
    while ((pos = buffer.find('\n')) != std::string::npos) {
        std::string line = buffer.substr(0, pos);
        buffer.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty() && recvSink_) recvSink_(line);
    }
}

void NetworkServer::IoLoop() {
    SOCKET ls = static_cast<SOCKET>(listenSock_);
    SOCKET us = static_cast<SOCKET>(udpSock_);

    while (running_.load()) {
        // Snapshot client sockets so select() runs without holding the lock.
        std::vector<SOCKET> clientSocks;
        {
            std::lock_guard<std::mutex> lock(clientsMutex_);
            for (auto& c : clients_) clientSocks.push_back(static_cast<SOCKET>(c.sock));
        }

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(ls, &readfds);
        if (us != INVALID_SOCKET) FD_SET(us, &readfds);
        for (SOCKET s : clientSocks) FD_SET(s, &readfds);

        timeval tv{0, 200000}; // 200ms poll so Stop() is responsive
        int r = select(0, &readfds, nullptr, nullptr, &tv);
        if (r <= 0) continue;

        // Accept new TCP clients.
        if (FD_ISSET(ls, &readfds)) {
            SOCKET cs = accept(ls, nullptr, nullptr);
            if (cs != INVALID_SOCKET) {
                std::lock_guard<std::mutex> lock(clientsMutex_);
                clients_.push_back({static_cast<uintptr_t>(cs), {}});
            }
        }

        // Read inbound UDP datagrams.
        if (us != INVALID_SOCKET && FD_ISSET(us, &readfds)) {
            char buf[2048];
            sockaddr_in from{};
            int fromLen = sizeof(from);
            int n = recvfrom(us, buf, sizeof(buf), 0,
                             reinterpret_cast<sockaddr*>(&from), &fromLen);
            if (n > 0) {
                std::string scratch; // UDP datagrams are self-contained lines
                DeliverLines(scratch, buf, n);
            }
        }

        // Read inbound TCP data, removing clients that have disconnected.
        for (SOCKET s : clientSocks) {
            if (!FD_ISSET(s, &readfds)) continue;
            char buf[2048];
            int n = recv(s, buf, sizeof(buf), 0);
            std::lock_guard<std::mutex> lock(clientsMutex_);
            auto it = std::find_if(clients_.begin(), clients_.end(),
                [&](const Client& c) { return static_cast<SOCKET>(c.sock) == s; });
            if (it == clients_.end()) continue;
            if (n <= 0) {
                closesocket(s);
                clients_.erase(it);
            } else {
                DeliverLines(it->rx, buf, n);
            }
        }
    }
}

void NetworkServer::Send(const std::string& line) {
    if (!running_.load()) return;

    // TCP: send to each client, dropping any that error out.
    {
        std::lock_guard<std::mutex> lock(clientsMutex_);
        for (auto it = clients_.begin(); it != clients_.end();) {
            SOCKET cs = static_cast<SOCKET>(it->sock);
            int sent = send(cs, line.c_str(), static_cast<int>(line.size()), 0);
            if (sent == SOCKET_ERROR) {
                closesocket(cs);
                it = clients_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // UDP: broadcast to the limited-broadcast address.
    if (udpSock_ != ~uintptr_t(0)) {
        sockaddr_in dest{};
        dest.sin_family = AF_INET;
        dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        dest.sin_port = htons(udpPort_);
        sendto(static_cast<SOCKET>(udpSock_), line.c_str(),
               static_cast<int>(line.size()), 0,
               reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
    }
}

int NetworkServer::ClientCount() const {
    std::lock_guard<std::mutex> lock(clientsMutex_);
    return static_cast<int>(clients_.size());
}

void NetworkServer::Stop() {
    if (!running_.exchange(false)) return;

    if (ioThread_.joinable()) ioThread_.join();

    {
        std::lock_guard<std::mutex> lock(clientsMutex_);
        for (auto& c : clients_) closesocket(static_cast<SOCKET>(c.sock));
        clients_.clear();
    }
    if (listenSock_ != ~uintptr_t(0)) { closesocket(static_cast<SOCKET>(listenSock_)); listenSock_ = ~uintptr_t(0); }
    if (udpSock_ != ~uintptr_t(0)) { closesocket(static_cast<SOCKET>(udpSock_)); udpSock_ = ~uintptr_t(0); }
}

} // namespace nmea
