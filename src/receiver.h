// UDP multicast receiver: joins a DIS multicast group and feeds Entity State
// PDUs into the EntityStore and Fire/Detonation PDUs into the EventStore, all
// on a background thread.
#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>
#include <atomic>
#include <thread>
#include <string>
#include <functional>
#include "dis.h"
#include "entity_store.h"
#include "event_store.h"

namespace net {

class MulticastReceiver {
public:
    // Optional callback invoked (on the receiver thread) with human-readable
    // status/error lines so the UI can surface them.
    std::function<void(const std::string&)> onStatus;

    MulticastReceiver(store::EntityStore& store,
                      store::EventStore& events,
                      std::string group = "239.1.2.3",
                      uint16_t port = 3000,
                      std::string iface = "0.0.0.0")
        : store_(store), events_(events), group_(std::move(group)), port_(port),
          iface_(std::move(iface)) {}

    ~MulticastReceiver() { stop(); }

    bool start() {
        if (running_) return true;

        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            status("WSAStartup failed"); return false;
        }

        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_ == INVALID_SOCKET) {
            status("socket() failed: " + std::to_string(WSAGetLastError()));
            WSACleanup(); return false;
        }

        // Allow multiple listeners on the same group/port on this host.
        BOOL reuse = TRUE;
        setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<char*>(&reuse), sizeof(reuse));

        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_ANY);
        local.sin_port = htons(port_);
        if (bind(sock_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) == SOCKET_ERROR) {
            status("bind() failed: " + std::to_string(WSAGetLastError()));
            closesocket(sock_); WSACleanup(); return false;
        }

        // Join the multicast group on the chosen interface.
        ip_mreq mreq{};
        inet_pton(AF_INET, group_.c_str(), &mreq.imr_multiaddr);
        inet_pton(AF_INET, iface_.c_str(), &mreq.imr_interface);
        if (setsockopt(sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                       reinterpret_cast<char*>(&mreq), sizeof(mreq)) == SOCKET_ERROR) {
            status("IP_ADD_MEMBERSHIP failed: " + std::to_string(WSAGetLastError()));
            closesocket(sock_); WSACleanup(); return false;
        }

        // 500 ms receive timeout so the thread can observe stop requests.
        DWORD tv = 500;
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<char*>(&tv), sizeof(tv));

        running_ = true;
        std::string on = "Listening on " + group_ + ":" + std::to_string(port_);
        if (iface_ != "0.0.0.0") on += " via " + iface_;
        status(on);
        thread_ = std::thread([this] { runLoop(); });
        return true;
    }

    void stop() {
        if (!running_) return;
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (sock_ != INVALID_SOCKET) { closesocket(sock_); sock_ = INVALID_SOCKET; }
        WSACleanup();
    }

    // Rejoin on a new group/port/interface (stops the current listener first).
    bool restart(std::string group, uint16_t port, std::string iface) {
        stop();
        group_ = std::move(group);
        port_  = port;
        iface_ = std::move(iface);
        packets_ = 0;
        return start();
    }

    const std::string& group() const { return group_; }
    uint16_t           port()  const { return port_; }
    const std::string& iface() const { return iface_; }

    uint64_t packetsReceived() const { return packets_.load(); }

private:
    void status(const std::string& s) { if (onStatus) onStatus(s); }

    void runLoop() {
        uint8_t buf[2048];
        while (running_) {
            sockaddr_in from{};
            int fromLen = sizeof(from);
            int n = recvfrom(sock_, reinterpret_cast<char*>(buf), sizeof(buf), 0,
                             reinterpret_cast<sockaddr*>(&from), &fromLen);
            if (n == SOCKET_ERROR) {
                int e = WSAGetLastError();
                if (e == WSAETIMEDOUT) continue;   // expected; check running_
                if (running_) status("recvfrom error: " + std::to_string(e));
                continue;
            }
            if (n <= 0) continue;

            packets_.fetch_add(1);
            dispatch(buf, size_t(n));
        }
    }

    // Route one datagram to the store that owns its PDU type. Unknown types
    // are counted (packets_) but otherwise ignored.
    void dispatch(const uint8_t* buf, size_t len) {
        switch (dis::pduType(buf, len)) {
            case dis::kPduTypeEntityState: {
                dis::EntityStatePdu p;
                if (dis::parseEntityState(buf, len, p)) store_.update(p);
                break;
            }
            case dis::kPduTypeFire: {
                dis::FirePdu p;
                if (dis::parseFire(buf, len, p)) events_.addFire(p);
                break;
            }
            case dis::kPduTypeDetonation: {
                dis::DetonationPdu p;
                if (dis::parseDetonation(buf, len, p)) events_.addDetonation(p);
                break;
            }
            default:
                break;
        }
    }

    store::EntityStore& store_;
    store::EventStore&  events_;
    std::string group_;
    uint16_t    port_;
    std::string iface_;
    SOCKET      sock_ = INVALID_SOCKET;
    std::thread thread_;
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> packets_{0};
};

} // namespace net
