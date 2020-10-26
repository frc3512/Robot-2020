// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#include "autonselector/AutonSelector.hpp"

#include <algorithm>
#include <fstream>
#include <memory>

#include <wpi/raw_ostream.h>

using namespace frc3512;
using namespace std::chrono_literals;

AutonSelector::AutonSelector(int port) : m_dsPort(port) {
    m_socket.Bind(port);
    m_socket.SetBlocking(false);

    // Retrieve stored autonomous index
#ifdef __FRC_ROBORIO__
    std::ifstream autonModeFile("/home/lvuser/autonMode.txt");
#else
    std::ifstream autonModeFile("autonMode.txt");
#endif
    if (autonModeFile.is_open()) {
        char autonMode;
        if (autonModeFile >> autonMode) {
            wpi::outs() << "AutonSelector: restored auton " << autonMode
                        << '\n';

            // Selection is stored as ASCII number in file
            m_curAutonMode = autonMode - '0';
        } else {
            wpi::outs() << "AutonSelector: failed restoring auton\n";
        }
    } else {
        wpi::outs() << "AutonSelector: failed opening autonMode.txt\n";
        m_curAutonMode = 0;
    }

    m_recvRunning = true;
    m_recvThread = std::thread([this] {
        while (m_recvRunning) {
            ReceiveFromDS();
            std::this_thread::sleep_for(100ms);
        }
    });
}

AutonSelector::~AutonSelector() {
    m_recvRunning = false;
    m_recvThread.join();
}

void AutonSelector::AddMode(std::string_view modeName,
                            std::function<void()> initFunc,
                            std::function<void()> periodicFunc) {
    m_autonModes.emplace_back(modeName, initFunc, periodicFunc);
}

void AutonSelector::SetMode(std::string_view modeName) {
    auto it =
        std::find_if(m_autonModes.begin(), m_autonModes.end(),
                     [&](const auto& i) { return std::get<0>(i) == modeName; });
    if (it != m_autonModes.cend()) {
        SetMode(std::distance(m_autonModes.begin(), it));
    }
}

void AutonSelector::SetMode(int mode) {
    m_curAutonMode = mode;

    char autonNum = '0' + m_curAutonMode;
    std::ofstream autonMode{"autonMode.txt"};
    autonMode << autonNum;
}

std::string_view AutonSelector::GetName(int mode) const {
    return std::get<0>(m_autonModes[mode]);
}

int AutonSelector::Size() const { return m_autonModes.size(); }

void AutonSelector::ExecAutonomousInit() {
    if (m_autonModes.size() == 0) {
        return;
    }

    // Retrieves correct autonomous routine and runs it
    std::get<1>(m_autonModes[m_curAutonMode])();
}

void AutonSelector::ExecAutonomousPeriodic() {
    if (m_autonModes.size() == 0) {
        return;
    }

    // Retrieves correct autonomous routine and runs it
    std::get<2>(m_autonModes[m_curAutonMode])();
}

void AutonSelector::SendToDS(Packet& packet) {
    // No locking needed here because this function is only used by
    // ReceiveFromDS(). Only other reads of m_dsIP and m_dsPort can occur at
    // this point.
    if (m_dsIP != 0) {
        m_socket.Send(packet, m_dsIP, m_dsPort);
    }
}

void AutonSelector::ReceiveFromDS() {
    // Send keepalive every 250ms
    auto time = steady_clock::now();
    if (time - m_prevTime > 250ms) {
        Packet packet;
        packet << std::string{"\r\n"};
        SendToDS(packet);

        m_prevTime = time;
    }

    if (m_socket.Receive(m_recvBuffer, 256, m_recvAmount, m_recvIP,
                         m_recvPort) == UdpSocket::Done) {
        std::string_view recvStr{m_recvBuffer, m_recvAmount};
        if (recvStr.substr(0, 9) == "connect\r\n") {
            {
                std::scoped_lock lock(m_ipMutex);
                m_dsIP = m_recvIP;
                m_dsPort = m_recvPort;
            }

            // Send a list of available autonomous modes
            Packet packet;
            packet << std::string{"autonList\r\n"};

            for (const auto& autonMode : m_autonModes) {
                packet << std::get<0>(autonMode);
            }

            SendToDS(packet);

            // Make sure driver knows which autonomous mode is selected
            packet.Clear();

            packet << std::string{"autonConfirmed\r\n"};
            if (m_autonModes.size() > 0) {
                packet << std::get<0>(m_autonModes[m_curAutonMode]);
            } else {
                packet << std::string{"N/A"};
            }

            SendToDS(packet);
        } else if (recvStr.substr(0, 13) == "autonSelect\r\n") {
            // Next byte after command is selection choice
            m_curAutonMode = m_recvBuffer[13];

            Packet packet;

            packet << std::string{"autonConfirmed\r\n"};
            packet << std::get<0>(m_autonModes[m_curAutonMode]);

            // Store newest autonomous choice to file for persistent storage
#ifdef __FRC_ROBORIO__
            std::ofstream autonModeFile("/home/lvuser/autonMode.txt",
                                        std::fstream::trunc);
#else
            std::ofstream autonModeFile("autonMode.txt", std::fstream::trunc);
#endif
            if (autonModeFile.is_open()) {
                // Selection is stored as ASCII number in file
                char autonNum = '0' + m_curAutonMode;

                if (autonModeFile << autonNum) {
                    wpi::outs() << "AutonSelector: autonSelect: wrote auton "
                                << autonNum << " to file\n";
                } else {
                    wpi::outs()
                        << "AutonSelector: autonSelect: failed writing auton "
                        << autonNum << " into open file\n";
                }
            } else {
                wpi::outs() << "AutonSelector: autonSelect: failed to open "
                               "autonMode.txt\n";
            }

            SendToDS(packet);
        }
    }
}
