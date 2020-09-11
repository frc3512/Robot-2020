// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "livegrapher/TcpSocket.hpp"

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <cstdio>
#include <system_error>

TcpSocket::TcpSocket() {
    m_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_fd == -1) {
        throw std::system_error(errno, std::system_category(), "TcpSocket");
    }
    DisableNagle();
}

TcpSocket::TcpSocket(int fd) : Socket(fd) { DisableNagle(); }

void TcpSocket::DisableNagle() {
    // Disable Nagle's algorithm
    int yes = 1;
    setsockopt(m_fd, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<char*>(&yes),
               sizeof(yes));
}
