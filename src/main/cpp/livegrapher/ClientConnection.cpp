// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#include "livegrapher/ClientConnection.hpp"

ClientConnection::ClientConnection(TcpSocket&& socket) {
    this->socket = std::move(socket);
}

void ClientConnection::SelectGraph(uint8_t id) { m_datasets |= 1 << id; }

void ClientConnection::UnselectGraph(uint8_t id) { m_datasets &= ~(1 << id); }

bool ClientConnection::IsGraphSelected(uint8_t id) {
    return m_datasets & (1 << id);
}

void ClientConnection::AddData(std::string_view data) {
    for (size_t i = 0; i < data.size(); ++i) {
        m_writeQueue.emplace_back(data[i]);
    }
}

bool ClientConnection::HasDataToWrite() const {
    return m_writeQueue.size() > 0;
}

bool ClientConnection::WriteToSocket() {
    int count = socket.Write({m_writeQueue.data(), m_writeQueue.size()});
    if (count == -1) {
        return false;
    } else {
        m_writeQueue.erase(m_writeQueue.begin(), m_writeQueue.begin() + count);
        return true;
    }
}
