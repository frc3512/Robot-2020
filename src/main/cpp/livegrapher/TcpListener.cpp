// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "livegrapher/TcpListener.hpp"

#include <arpa/inet.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <cstdio>
#include <cstring>
#include <system_error>

TcpListener::TcpListener(uint16_t port) {
    m_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_fd == -1) {
        throw std::system_error(errno, std::system_category(), "TcpListener");
    }
    DisableNagle();

    // Allow rebinding to the socket later if the connection is interrupted
    int optval = 1;
    setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    sockaddr_in servAddr;

    std::memset(&servAddr, 0, sizeof(sockaddr_in));

    // Set up the listener sockaddr_in struct
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = 0;
    servAddr.sin_port = htons(port);

    // Bind the socket to the listener sockaddr_in
    if (bind(m_fd, reinterpret_cast<sockaddr*>(&servAddr),
             sizeof(sockaddr_in)) != 0) {
        throw std::system_error(errno, std::system_category(), "bind");
    }

    // Listen on the socket for incoming connections
    if (listen(m_fd, 5) != 0) {
        throw std::system_error(errno, std::system_category(), "listen");
    }

    // Make sure we aren't killed by SIGPIPE
    signal(SIGPIPE, SIG_IGN);
}

TcpSocket TcpListener::Accept() {
    unsigned int cliLen;
    sockaddr_in cliAddr;

    cliLen = sizeof(cliAddr);

    // Accept a new connection
    TcpSocket newSocket{
        accept(m_fd, reinterpret_cast<sockaddr*>(&cliAddr), &cliLen)};

    // Make sure that the file descriptor is valid
    if (newSocket.m_fd == -1) {
        throw std::system_error(errno, std::system_category(), "accept");
    }

    newSocket.SetBlocking(false);

    return newSocket;
}
