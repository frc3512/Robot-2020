// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include "livegrapher/Socket.hpp"

class TcpSocket : public Socket {
public:
    TcpSocket();

    /**
     * Constructs a TcpSocket.
     *
     * @param fd The socket file descriptor to assume ownership of.
     */
#ifdef _WIN32
    explicit TcpSocket(SOCKET fd);
#else
    explicit TcpSocket(int fd);
#endif

    TcpSocket(TcpSocket&&) = default;
    TcpSocket& operator=(TcpSocket&&) = default;

private:
    friend class TcpListener;

    /**
     * Disable Nagle's algorithm.
     *
     * See https://en.wikipedia.org/wiki/Nagle%27s_algorithm.
     */
    void DisableNagle();
};
