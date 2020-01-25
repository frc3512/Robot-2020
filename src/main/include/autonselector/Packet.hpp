// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <string>
#include <vector>

namespace frc3512 {

/**
 * Utility class to build blocks of data to transfer over the network
 */
class Packet {
public:
    Packet() = default;

    // Append data to the end of the packet
    void append(const void* data, size_t sizeInBytes);

    // Empty the packet
    void clear();

    /* Get a pointer to the data contained in the packet
     *
     * Warning: the returned pointer may become invalid after you append data
     * to the packet, therefore it should never be stored.
     *
     * The return pointer is nullptr if the packet is empty.
     */
    const void* getData() const;

    /* Get the size of the data contained in the packet in bytes
     *
     * This function returns the number of bytes pointed to by what getData
     * returns.
     */
    size_t getDataSize() const;

public:
    // Overloads of operator<< to write data into the packet
    Packet& operator>>(bool& data);
    Packet& operator>>(int8_t& data);
    Packet& operator>>(uint8_t& data);
    Packet& operator>>(int16_t& data);
    Packet& operator>>(uint16_t& data);
    Packet& operator>>(int32_t& data);
    Packet& operator>>(uint32_t& data);
    Packet& operator>>(int64_t& data);
    Packet& operator>>(uint64_t& data);
    Packet& operator>>(float& data);
    Packet& operator>>(double& data);
    Packet& operator>>(std::string& data);
    Packet& operator<<(bool data);
    Packet& operator<<(int8_t data);
    Packet& operator<<(uint8_t data);
    Packet& operator<<(int16_t data);
    Packet& operator<<(uint16_t data);
    Packet& operator<<(int32_t data);
    Packet& operator<<(uint32_t data);
    Packet& operator<<(int64_t data);
    Packet& operator<<(uint64_t data);
    Packet& operator<<(float data);
    Packet& operator<<(double data);
    Packet& operator<<(const std::string& data);

private:
    // Data stored in the packet
    std::vector<char> m_packetData;

    // Reading state of the packet
    bool m_isValid = true;

    // Current reading position in the packet
    size_t m_readPos = 0;

    bool operator==(const Packet& right) const;
    bool operator!=(const Packet& right) const;

    // Checks if the packet can extract a given number of bytes
    bool CheckSize(size_t size);
};

}  // namespace frc3512
