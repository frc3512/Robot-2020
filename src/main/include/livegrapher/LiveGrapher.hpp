// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <atomic>
#include <chrono>
#include <map>
#include <string>
#include <thread>
#include <vector>

#if defined(__FRC_ROBORIO__)
#include <wpi/mutex.h>
#else
#include <mutex>

namespace wpi {
using mutex = ::std::mutex;
}  // namespace wpi
#endif

#include "livegrapher/ClientConnection.hpp"
#include "livegrapher/SocketSelector.hpp"
#include "livegrapher/TcpListener.hpp"

/**
 * The host for the LiveGrapher real-time graphing application.
 *
 * Usage:
 *
 * The LiveGrapher interface is started upon object initialization.
 *
 * Call AddData() to send data over the network to a LiveGrapher client.
 *
 * The time value in each data pair is handled internally.
 *
 * Example:
 *     LiveGrapher grapher{3513};
 *
 *     void TeleopPeriodic() override {
 *         grapher.GraphData("PID0", frisbeeShooter.getRPM());
 *         grapher.GraphData("PID1", frisbeeShooter.getTargetRPM());
 *     }
 */
class LiveGrapher {
public:
    /**
     * Constructs a LiveGrapher host.
     *
     * @param port The port on which to listen for new clients.
     */
    explicit LiveGrapher(uint16_t port);

    ~LiveGrapher();

    /**
     * Send data (y value) for a given dataset to remote client.
     *
     * The current time is sent as the x value.
     *
     * @param dataset The name of the dataset to which the value belongs.
     * @param value   The y value.
     */
    void AddData(const std::string& dataset, float value);

    /**
     * Send time (x value) and data (y value) for a given dataset to remote
     * client.
     *
     * @param dataset The name of the dataset to which the value belongs.
     * @param time    The x value.
     * @param value   The y value.
     */
    void AddData(const std::string& dataset, std::chrono::milliseconds time,
                 float value);

private:
    std::thread m_thread;
    wpi::mutex m_connListMutex;
    std::atomic<bool> m_isRunning{false};
    TcpListener m_listener;
    SocketSelector m_selector;

    // Sorted by graph name instead of ID because the user passes in a string.
    // (They don't know the ID.) This makes graph ID lookups take O(log n).
    std::map<std::string, uint8_t> m_graphList;

    std::vector<ClientConnection> m_connList;

    /**
     * Extract the packet type from the ID field of a received client packet.
     *
     * @param id The client packet ID field.
     */
    static constexpr uint8_t PacketType(uint8_t id) {
        // Masks two high-order bits
        return id & 0xC0;
    }

    /**
     * Extract the graph ID from a host connect or disconnect packet's ID field.
     *
     * @param id The client packet ID field.
     */
    static constexpr uint8_t GraphID(uint8_t id) {
        // Masks six low-order bits
        return id & 0x3F;
    }

    /**
     * Send time (x value) and data (y value) for a given dataset to remote
     * client.
     *
     * @param dataset The name of the dataset to which the value belongs.
     * @param time    The x value.
     * @param value   The y value.
     */
    void AddDataImpl(const std::string& dataset, std::chrono::milliseconds time,
                     float value);

    /**
     * Function for thread that reads and writes graph data.
     */
    void ThreadMain();

    /**
     * Read packets from the given client.
     *
     * @param conn The client connection.
     * @return 0 if the read succeeded and -1 if it failed.
     */
    int ReadPackets(ClientConnection& conn);
};
