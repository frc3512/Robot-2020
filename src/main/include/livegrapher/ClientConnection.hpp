// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <vector>

#include "livegrapher/TcpSocket.hpp"

/**
 * Wrapper around graph client socket.
 */
class ClientConnection {
public:
    TcpSocket socket;

    /**
     * Constructs a client connection.
     *
     * @param socket The client socket.
     */
    explicit ClientConnection(TcpSocket&& socket);

    ClientConnection(ClientConnection&&) = default;
    ClientConnection& operator=(ClientConnection&&) = default;

    /**
     * Select a graph so the data for it will be sent.
     *
     * @param id The ID of the graph to select.
     */
    void SelectGraph(uint8_t id);

    /**
     * Unselect a graph so the data for it will no longer be sent.
     *
     * @param id The ID of the graph to unselect.
     */
    void UnselectGraph(uint8_t id);

    /**
     * Returns true if the given graph is selected.
     *
     * @param id The ID of the graph.
     */
    bool IsGraphSelected(uint8_t id);

    /**
     * Add data to write queue.
     *
     * @param data The data to enqueue.
     */
    void AddData(std::string_view data);

    /**
     * Returns true if there's data in the write queue.
     */
    bool HasDataToWrite() const;

    /**
     * Attempt to send queued data on socket.
     *
     * @return True if write succeeded. This doesn't necessarily mean all data
     *         was sent.
     */
    bool WriteToSocket();

private:
    std::vector<char> m_writeQueue;

    // A bitfield representing the selection state of each graph ID. The LSB is
    // the selection state of graph ID 0 and the MSB is the selection state of
    // graph ID 63. Graph IDs are 6 bits wide, so there are 2^6 = 64 possible
    // graph IDs.
    uint64_t m_datasets = 0;
};
