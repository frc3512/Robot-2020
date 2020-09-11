// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include "livegrapher/TcpSocket.hpp"

class TcpListener : public TcpSocket {
public:
    /**
     * Constructs a TcpListener.
     *
     * @param port The port on which to listen for new TCP connections.
     */
    explicit TcpListener(uint16_t port);

    TcpListener(TcpListener&&) = default;
    TcpListener& operator=(TcpListener&&) = default;

    /**
     * Accepts a pending TCP connection and returns the TcpSocket.
     *
     * This blocks until a connection is pending.
     */
    TcpSocket Accept();
};
