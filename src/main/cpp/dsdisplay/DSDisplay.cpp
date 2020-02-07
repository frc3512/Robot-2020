// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#include "dsdisplay/DSDisplay.hpp"

#include <cstring>
#include <fstream>
#include <memory>

#include <frc/Filesystem.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <wpi/Twine.h>
#include <wpi/raw_ostream.h>

using namespace frc3512;
using namespace std::chrono_literals;

DSDisplay::DSDisplay(int port) : m_dsPort(port) {
    m_socket.bind(port);
    m_socket.setBlocking(false);

    // Retrieve stored autonomous index
    wpi::SmallString<64> path;
    frc::filesystem::GetOperatingDirectory(path);
    wpi::sys::path::append(path, "autonMode.txt");
    std::ifstream autonModeFile(wpi::Twine{path}.str());
    if (autonModeFile.is_open()) {
        if (autonModeFile >> m_curAutonMode) {
            wpi::outs() << "dsdisplay: restored auton " << m_curAutonMode
                        << "\n";

            // Selection is stored as ASCII number in file
            m_curAutonMode -= '0';
        } else {
            wpi::errs() << "dsdisplay: failed restoring auton\n";
        }
    } else {
        wpi::errs() << "dsdisplay: failed opening autonMode.txt\n";
        m_curAutonMode = 0;
    }

    m_recvRunning = true;
    m_recvThread = std::thread([this] {
        while (m_recvRunning) {
            ReceiveFromDS();
            std::this_thread::sleep_for(10ms);
        }
    });
}

DSDisplay::~DSDisplay() {
    m_recvRunning = false;
    m_recvThread.join();
}

void DSDisplay::Clear() { m_packet.clear(); }

void DSDisplay::AddData(std::string ID, StatusLight data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('c');
    m_packet << ID;
    m_packet << static_cast<int8_t>(data);
}

void DSDisplay::AddData(std::string ID, bool data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('c');
    m_packet << ID;

    if (data == true) {
        m_packet << static_cast<int8_t>(DSDisplay::active);
    } else {
        m_packet << static_cast<int8_t>(DSDisplay::inactive);
    }
}

void DSDisplay::AddData(std::string ID, int8_t data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('c');
    m_packet << ID;
    m_packet << data;
}

void DSDisplay::AddData(std::string ID, int32_t data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('i');
    m_packet << ID;
    m_packet << data;
}

void DSDisplay::AddData(std::string ID, std::string data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('s');
    m_packet << ID;
    m_packet << data;
}

void DSDisplay::AddData(std::string ID, double data) {
    // If packet is empty, add "display\r\n" header to packet
    if (m_packet.getData() == nullptr) {
        // doubles are converted to strings because VxWorks messes up floating
        // point values over the network.
        m_packet << std::string("display\r\n");
    }

    m_packet << static_cast<int8_t>('s');
    m_packet << ID;
    m_packet << std::to_string(data);
}

void DSDisplay::SendToDS() {
    uint32_t dsIP;
    uint16_t dsPort;

    {
        std::lock_guard lock(m_ipMutex);
        dsIP = m_dsIP;
        dsPort = m_dsPort;
    }

    if (dsIP != 0) {
        m_socket.send(m_packet, dsIP, dsPort);
        Clear();
    }
}

void DSDisplay::AddAutoMethod(std::string methodName,
                              std::function<void()> initFunc,
                              std::function<void()> periodicFunc) {
    m_autonModes.emplace_back(methodName, initFunc, periodicFunc);
}

void DSDisplay::DeleteAllMethods() { m_autonModes.clear(); }

std::string DSDisplay::GetAutonomousMode() const {
    return std::get<0>(m_autonModes[m_curAutonMode]);
}

void DSDisplay::ExecAutonomousInit() {
    // Retrieves correct autonomous routine and runs it
    std::get<1>(m_autonModes[m_curAutonMode])();
}

void DSDisplay::ExecAutonomousPeriodic() {
    // Retrieves correct autonomous routine and runs it
    std::get<2>(m_autonModes[m_curAutonMode])();
}

void DSDisplay::SendToDS(Packet& packet) {
    // No locking needed here because this function is only used by
    // ReceiveFromDS(). Only other reads of m_dsIP and m_dsPort can occur at
    // this point.
    if (m_dsIP != 0) {
        m_socket.send(packet, m_dsIP, m_dsPort);
        m_packet.clear();
    }
}

void DSDisplay::ReceiveFromDS() {
    // Send keepalive every 250ms
    auto time = steady_clock::now();
    if (time - m_prevTime > 250ms) {
        Packet packet;
        packet << static_cast<std::string>("\r\n");
        SendToDS(packet);

        m_prevTime = time;
    }

    if (m_socket.receive(m_recvBuffer, 256, m_recvAmount, m_recvIP,
                         m_recvPort) == UdpSocket::Done) {
        if (std::strncmp(m_recvBuffer, "connect\r\n", 9) == 0) {
            {
                std::lock_guard lock(m_ipMutex);
                m_dsIP = m_recvIP;
                m_dsPort = m_recvPort;
            }

            // Send GUI element file to DS

            Packet packet;
            packet << static_cast<std::string>("guiCreate\r\n");

            // Open the file
#ifdef __FRC_ROBORIO__
            std::ifstream guiFile("GUISettings.txt", std::ifstream::binary);
#else
            std::ifstream guiFile("GUISettings.txt", std::ifstream::binary);
#endif

            if (guiFile.is_open()) {
                // Get its length
                guiFile.seekg(0, guiFile.end);
                unsigned int fileSize = guiFile.tellg();
                guiFile.seekg(0, guiFile.beg);

                // Send the length
                packet << static_cast<uint32_t>(fileSize);

                // Allocate a buffer for the file
                auto tempBuf = std::make_unique<char[]>(fileSize);

                // Send the data
                guiFile.read(tempBuf.get(), fileSize);
                packet.append(tempBuf.get(), fileSize);

                guiFile.close();
            }

            SendToDS(packet);

            // Send a list of available autonomous modes
            packet.clear();

            packet << static_cast<std::string>("autonList\r\n");

            for (unsigned int i = 0; i < m_autonModes.size(); i++) {
                packet << std::get<0>(m_autonModes[i]);
            }

            SendToDS(packet);

            // Make sure driver knows which autonomous mode is selected
            packet.clear();

            packet << static_cast<std::string>("autonConfirmed\r\n");
            packet << std::get<0>(m_autonModes[m_curAutonMode]);

            SendToDS(packet);
        } else if (std::strncmp(m_recvBuffer, "autonSelect\r\n", 13) == 0) {
            // Next byte after command is selection choice
            m_curAutonMode = m_recvBuffer[13];

            Packet packet;

            packet << static_cast<std::string>("autonConfirmed\r\n");
            packet << std::get<0>(m_autonModes[m_curAutonMode]);

            // Store newest autonomous choice to file for persistent storage
            wpi::SmallString<64> path;
            frc::filesystem::GetOperatingDirectory(path);
            wpi::sys::path::append(path, "autonMode.txt");
            std::ofstream autonModeFile(wpi::Twine{path}.str(),
                                        std::fstream::trunc);
            if (autonModeFile.is_open()) {
                // Selection is stored as ASCII number in file
                char autonNum = '0' + m_curAutonMode;

                if (autonModeFile << autonNum) {
                    wpi::outs() << "dsdisplay: autonSelect: wrote auton "
                                << autonNum << " to file\n";
                } else {
                    wpi::errs()
                        << "dsdisplay: autonSelect: failed writing auton "
                        << autonNum << " into open file\n";
                }
            } else {
                wpi::errs()
                    << "dsdisplay: autonSelect: failed to open autonMode.txt\n";
            }

            SendToDS(packet);
        }
    }
}
