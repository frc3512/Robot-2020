// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "livegrapher/TcpSocket.hpp"

#ifdef _WIN32
#define _WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <winsock2.h>

#else
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#endif

#include <cstdio>
#include <system_error>

TcpSocket::TcpSocket() {
    m_fd = socket(AF_INET, SOCK_STREAM, 0);
#ifdef _WIN32
    if (m_fd == INVALID_SOCKET) {
#else
    if (m_fd == -1) {
#endif
        throw std::system_error(errno, std::system_category(), "TcpSocket");
    }
    DisableNagle();
}

#ifdef _WIN32
TcpSocket::TcpSocket(SOCKET fd) : Socket(fd) { DisableNagle(); }
#else
TcpSocket::TcpSocket(int fd) : Socket(fd) { DisableNagle(); }
#endif

void TcpSocket::DisableNagle() {
    // Disable Nagle's algorithm
    int yes = 1;
    setsockopt(m_fd, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<char*>(&yes),
               sizeof(yes));
}
