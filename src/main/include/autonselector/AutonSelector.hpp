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
 * The Autonomous Selector is a program we wrote and a protocol we invented to
 * facilitate running multiple autonomous modes, each being selectable from the
 * Driver Station.
 *
 * USAGE:
 * 1) Instantiate AutonSelector with the port on which communications will be
 *    received (probably 1130).
 * 2) Call AddMethod() with each available autonomous mode.
 * 3) Call ExecAutonomousInit() and ExecAutonomousPeriodic() in the init and
 *    periodic functions of the robot class.
 */
class AutonSelector {
public:
    explicit AutonSelector(int port);
    ~AutonSelector();

    AutonSelector(const AutonSelector&) = delete;
    AutonSelector& operator=(const AutonSelector&) = delete;

    /**
     * Adds an autonomous function.
     *
     * @param modeName Name of autonomous mode.
     * @param initFunc Init() function for autonomous mode that will run in
     *                 AutonomousInit().
     * @param periodicFunc Periodic() function for autonomous mode that will run
     *                     in AutonomousPeriodic().
     */
    void AddMode(std::string_view modeName, std::function<void()> initFunc,
                 std::function<void()> periodicFunc);

    /**
     * Sets the selected autonomous mode.
     *
     * @param modeName Name of autonomous mode.
     */
    void SetMode(std::string_view modeName);

    /**
     * Sets the auton mode for the autonomous unit tests
     *
     * @param mode Numbered position of autonomous mode (numbers are assigned to
     *             autonomous mode in the order they are added starting from
     *             zero).
     */
    void SetMode(int mode);

    /**
     * Returns autonomous mode name for the given index.
     *
     * @param mode Numbered position of autonomous mode (numbers are assigned to
     *             autonomous mode in the order they are added starting from
     *             zero).
     */
    std::string_view GetName(int mode) const;

    /**
     * Returns number of registered autonomous modes.
     */
    int Size() const;

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
    std::atomic<char> m_curAutonMode;

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
