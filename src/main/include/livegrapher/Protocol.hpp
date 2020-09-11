// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <string>

// LiveGrapher wire protocol. See README.md in the root directory of this
// project for protocol documentation.

struct [[gnu::packed]] HostPacket {
    uint8_t ID;
};

constexpr uint8_t kHostConnectPacket = 0b00 << 6;
constexpr uint8_t kHostDisconnectPacket = 0b01 << 6;
constexpr uint8_t kHostListPacket = 0b10 << 6;

struct [[gnu::packed]] ClientDataPacket {
    uint8_t ID;
    uint64_t x;
    float y;
};

struct ClientListPacket {
    uint8_t ID;
    uint8_t length;
    std::string name;
    uint8_t eof;
};

constexpr uint8_t kClientDataPacket = 0b00 << 6;
constexpr uint8_t kClientListPacket = 0b01 << 6;
