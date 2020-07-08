// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <vector>

#include <wpi/mutex.h>

#include "autonselector/Packet.hpp"
#include "autonselector/UdpSocket.hpp"

namespace frc3512 {
/**
 * This class allows you to pack data into an SFML packet and send it to an
 * application on the DriverStation that displays it in a GUI.
 *
 * USAGE:
 * 1) Instantiate AutonSelector with the port on which communications will be
 *    received (probably 1130).
 * 2) Call several variations of AddData().
 * 3) After all data is packed, call SendToDS() to send the data to the Driver
 *    Station.
 */

class AutonSelector {
public:
    explicit AutonSelector(int port);
    ~AutonSelector();

    AutonSelector(const AutonSelector&) = delete;
    AutonSelector& operator=(const AutonSelector&) = delete;

    /**
     * Add an autonomous function.
     */
    void AddAutoMethod(std::string_view methodName,
                       std::function<void()> initFunc,
                       std::function<void()> periodicFunc);

    /**
     * Remove all autonomous functions.
     */
    void DeleteAllMethods();

    /**
     * Returns the name of the currently selected autonomous function.
     */
    std::string GetAutonomousMode() const;

    /**
     * Runs autonomous init function currently selected.
     */
    void ExecAutonomousInit();

    /**
     * Runs autonomous periodic function currently selected.
     */
    void ExecAutonomousPeriodic();

private:
    using steady_clock = std::chrono::steady_clock;

    UdpSocket m_socket;  // socket for sending data to Driver Station
    uint32_t m_dsIP;     // IP address of Driver Station
    int m_dsPort;        // port to which to send data

    // Rate-limits keepalive
    steady_clock::time_point m_prevTime = steady_clock::now();

    // Stores IP address temporarily during receive
    uint32_t m_recvIP;

    // Stores port temporarily during receive
    uint16_t m_recvPort = 0;

    // Buffer for Driver Station requests
    char m_recvBuffer[256];

    // Holds number of bytes received from Driver Station
    size_t m_recvAmount = 0;

    std::vector<
        std::tuple<std::string, std::function<void()>, std::function<void()>>>
        m_autonModes;
    char m_curAutonMode;

    std::thread m_recvThread;
    wpi::mutex m_ipMutex;
    std::atomic<bool> m_recvRunning{false};

    /**
     * Sends the given packet to the Driver Station.
     */
    void SendToDS(Packet& packet);

    /**
     * Receives control commands from Driver Station and processes them.
     */
    void ReceiveFromDS();
};
}  // namespace frc3512
