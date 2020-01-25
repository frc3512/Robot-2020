// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "autonselector/Packet.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>

#include <cstring>

using namespace frc3512;

void Packet::append(const void* data, size_t sizeInBytes) {
    if (data && (sizeInBytes > 0)) {
        size_t start = m_packetData.size();
        m_packetData.resize(start + sizeInBytes);
        std::memcpy(&m_packetData[start], data, sizeInBytes);
    }
}

void Packet::clear() { m_packetData.clear(); }

const void* Packet::getData() const {
    if (!m_packetData.empty()) {
        return &m_packetData[0];
    } else {
        return nullptr;
    }
}

size_t Packet::getDataSize() const { return m_packetData.size(); }

Packet& Packet::operator>>(bool& data) {
    uint8_t value;
    *this >> value;
    if (m_isValid) data = (value != 0);
    return *this;
}

Packet& Packet::operator>>(int8_t& data) {
    if (CheckSize(sizeof(data))) {
        data = *reinterpret_cast<const int8_t*>(&m_packetData[m_readPos]);
        m_readPos += sizeof(data);
    }

    return *this;
}

Packet& Packet::operator>>(uint8_t& data) {
    if (CheckSize(sizeof(data))) {
        data = *reinterpret_cast<const uint8_t*>(&m_packetData[m_readPos]);
        m_readPos += sizeof(data);
    }

    return *this;
}

Packet& Packet::operator>>(int16_t& data) {
    if (CheckSize(sizeof(data))) {
        data =
            ntohs(*reinterpret_cast<const int16_t*>(&m_packetData[m_readPos]));
        m_readPos += sizeof(data);
    }

    return *this;
}

Packet& Packet::operator>>(uint16_t& data) {
    if (CheckSize(sizeof(data))) {
        data =
            ntohs(*reinterpret_cast<const uint16_t*>(&m_packetData[m_readPos]));
        m_readPos += sizeof(data);
    }

    return *this;
}

Packet& Packet::operator>>(int32_t& data) {
    if (CheckSize(sizeof(data))) {
        data =
            ntohl(*reinterpret_cast<const int32_t*>(&m_packetData[m_readPos]));
        m_readPos += sizeof(data);
    }

    return *this;
}

Packet& Packet::operator>>(uint32_t& data) {
    if (CheckSize(sizeof(data))) {
        data =
            ntohl(*reinterpret_cast<const uint32_t*>(&m_packetData[m_readPos]));
        m_readPos += sizeof(data);
    }

    return *this;
}

Packet& Packet::operator>>(int64_t& data) {
    if (CheckSize(sizeof(data))) {
        data =
            ntohl(*reinterpret_cast<const int64_t*>(&m_packetData[m_readPos]));
        m_readPos += sizeof(data);
    }

    return *this;
}

Packet& Packet::operator>>(uint64_t& data) {
    if (CheckSize(sizeof(data))) {
        data =
            ntohl(*reinterpret_cast<const uint64_t*>(&m_packetData[m_readPos]));
        m_readPos += sizeof(data);
    }

    return *this;
}

Packet& Packet::operator>>(float& data) {
    if (CheckSize(sizeof(data))) {
        data = *reinterpret_cast<const float*>(&m_packetData[m_readPos]);
        m_readPos += sizeof(data);
    }

    return *this;
}

Packet& Packet::operator>>(double& data) {
    if (CheckSize(sizeof(data))) {
        data = *reinterpret_cast<const double*>(&m_packetData[m_readPos]);
        m_readPos += sizeof(data);
    }

    return *this;
}

Packet& Packet::operator>>(std::string& data) {
    // First extract string length
    uint32_t length = 0;
    *this >> length;

    data.clear();
    if ((length > 0) && CheckSize(length)) {
        // Then extract characters
        data.assign(&m_packetData[m_readPos], length);

        // Update reading position
        m_readPos += length;
    }

    return *this;
}

Packet& Packet::operator<<(bool data) {
    *this << static_cast<uint8_t>(data);
    return *this;
}

Packet& Packet::operator<<(int8_t data) {
    append(&data, sizeof(data));
    return *this;
}

Packet& Packet::operator<<(uint8_t data) {
    append(&data, sizeof(data));
    return *this;
}

Packet& Packet::operator<<(int16_t data) {
    int16_t toWrite = htons(data);
    append(&toWrite, sizeof(toWrite));
    return *this;
}

Packet& Packet::operator<<(uint16_t data) {
    uint16_t toWrite = htons(data);
    append(&toWrite, sizeof(toWrite));
    return *this;
}

Packet& Packet::operator<<(int32_t data) {
    int32_t toWrite = htonl(data);
    append(&toWrite, sizeof(toWrite));
    return *this;
}

Packet& Packet::operator<<(uint32_t data) {
    uint32_t toWrite = htonl(data);
    append(&toWrite, sizeof(toWrite));
    return *this;
}

Packet& Packet::operator<<(int64_t data) {
    int64_t toWrite = htonl(data);
    append(&toWrite, sizeof(toWrite));
    return *this;
}

Packet& Packet::operator<<(uint64_t data) {
    uint64_t toWrite = htonl(data);
    append(&toWrite, sizeof(toWrite));
    return *this;
}

Packet& Packet::operator<<(float data) {
    append(&data, sizeof(data));
    return *this;
}

Packet& Packet::operator<<(double data) {
    append(&data, sizeof(data));
    return *this;
}

Packet& Packet::operator<<(const std::string& data) {
    // First insert string length
    uint32_t length = static_cast<uint32_t>(data.size());
    *this << length;

    // Then insert characters
    if (length > 0) {
        append(data.c_str(), length * sizeof(std::string::value_type));
    }

    return *this;
}

bool Packet::CheckSize(size_t size) {
    m_isValid = m_isValid && (m_readPos + size <= m_packetData.size());

    return m_isValid;
}
