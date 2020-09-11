// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#include "livegrapher/LiveGrapher.hpp"

#include <arpa/inet.h>

#include <algorithm>
#include <cstring>

#include "livegrapher/Protocol.hpp"

LiveGrapher::LiveGrapher(uint16_t port) : m_listener{port} {
    m_selector.Add(m_listener, SocketSelector::kRead);

    m_isRunning = true;
    m_thread = std::thread([this] { ThreadMain(); });
}

LiveGrapher::~LiveGrapher() {
    m_isRunning = false;
    m_selector.Cancel();
    m_thread.join();
}

void LiveGrapher::AddData(const std::string& dataset, float value) {
    // HACK: The dataset argument uses const std::string& instead of
    // std::string_view because std::map doesn't have a find(std::string_view)
    // overload.

    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using std::chrono::steady_clock;

    auto currentTime =
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch());

    AddData(dataset, currentTime, value);
}

void LiveGrapher::AddData(const std::string& dataset,
                          std::chrono::milliseconds time, float value) {
    // HACK: The dataset argument uses const std::string& instead of
    // std::string_view because std::map doesn't have a find(std::string_view)
    // overload.

    // This will only work if ints are the same size as floats
    static_assert(sizeof(float) == sizeof(uint32_t),
                  "float isn't 32 bits long");

    auto i = m_graphList.find(dataset);
    uint8_t id = i->second;

    if (i == m_graphList.end()) {
        m_graphList.emplace(dataset, m_graphList.size());
    }

    ClientDataPacket packet;
    packet.ID = kClientDataPacket | id;

    // Change to network byte order
    // Swap bytes in x, and copy into the payload struct
    uint64_t timeMs = time.count();
    timeMs = htobe64(timeMs);
    std::memcpy(&packet.x, &timeMs, sizeof(timeMs));

    // Swap bytes in y, and copy into the payload struct
    uint32_t ytmp;
    std::memcpy(&ytmp, &value, sizeof(ytmp));
    ytmp = htonl(ytmp);
    std::memcpy(&packet.y, &ytmp, sizeof(ytmp));

    bool restartSelect = false;

    {
        std::scoped_lock lock(m_connListMutex);

        // Send the point to connected clients
        for (auto& conn : m_connList) {
            if (conn.IsGraphSelected(id)) {
                conn.AddData(
                    {reinterpret_cast<char*>(&packet), sizeof(packet)});
                restartSelect = true;
            }
        }
    }

    if (restartSelect) {
        // Restart select() with new data queued for write so it gets sent out
        m_selector.Cancel();
    }
}

void LiveGrapher::ThreadMain() {
    while (m_isRunning) {
        {
            std::scoped_lock lock(m_connListMutex);

            // Mark select on write for sockets with data queued
            for (const auto& conn : m_connList) {
                if (conn.HasDataToWrite()) {
                    m_selector.Add(conn.socket, SocketSelector::kWrite);
                }
            }
        }

        try {
            if (!m_selector.Select()) {
                continue;
            }
        } catch (const std::system_error&) {
            // If select() failed, one of the client socket descriptors is
            // probably bad. We can't determine which, so we'll close all client
            // connections. It's better than crashing the host.
            for (const auto& conn : m_connList) {
                m_selector.Remove(conn.socket, SocketSelector::kRead |
                                                   SocketSelector::kWrite);
            }
            m_connList.clear();
            continue;
        }

        {
            std::scoped_lock lock(m_connListMutex);

            auto conn = m_connList.begin();
            while (conn != m_connList.end()) {
                if (m_selector.IsReadReady(conn->socket)) {
                    // If the read failed, remove the socket from the selector
                    // and close the connection
                    if (ReadPackets(*conn) == -1) {
                        m_selector.Remove(
                            conn->socket,
                            SocketSelector::kRead | SocketSelector::kWrite);
                        conn = m_connList.erase(conn);
                        continue;
                    }
                }

                if (m_selector.IsWriteReady(conn->socket)) {
                    // If the write failed, remove the socket from the selector
                    // and close the connection
                    if (!conn->WriteToSocket()) {
                        m_selector.Remove(
                            conn->socket,
                            SocketSelector::kRead | SocketSelector::kWrite);
                        conn = m_connList.erase(conn);
                        continue;
                    }

                    if (!conn->HasDataToWrite()) {
                        m_selector.Remove(conn->socket, SocketSelector::kWrite);
                    }
                }

                ++conn;
            }
        }

        if (m_selector.IsReadReady(m_listener)) {
            auto socket = m_listener.Accept();
            m_selector.Add(socket, SocketSelector::kRead);

            std::scoped_lock lock(m_connListMutex);
            m_connList.emplace_back(std::move(socket));
        }
    }
}

int LiveGrapher::ReadPackets(ClientConnection& conn) {
    char id;

    if (!conn.socket.Read(&id, 1)) {
        return -1;
    }

    switch (PacketType(id)) {
        case kHostConnectPacket:
            // Start sending data for the graph specified by the ID
            conn.SelectGraph(GraphID(id));
            break;
        case kHostDisconnectPacket:
            // Stop sending data for the graph specified by the ID
            conn.UnselectGraph(GraphID(id));
            break;
        case kHostListPacket:
            // 255 is the max graph name length
            char buf[1 + 1 + 255 + 1];

            // A graph count is compared against instead of the graph ID for
            // terminating list traversal because the std::map is sorted by
            // graph name instead of the ID. Since, the IDs are not necessarily
            // in order, early traversal termination could occur.
            size_t graphCount = 0;
            for (const auto& [graph, id] : m_graphList) {
                buf[0] = kClientListPacket | id;
                buf[1] = graph.length();
                std::strncpy(&buf[2], graph.c_str(), graph.length());

                // Is this the last element in the list?
                if (graphCount + 1 == m_graphList.size()) {
                    buf[2 + buf[1]] = 1;
                } else {
                    buf[2 + buf[1]] = 0;
                }

                // Send graph name. The data size is computed explicitly here
                // because the buffer string's current length may be larger than
                // that.
                conn.AddData({buf, 1 + 1 + graph.length() + 1});

                graphCount++;
            }
            break;
    }

    return 0;
}
