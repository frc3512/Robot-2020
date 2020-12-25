// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "livegrapher/Socket.hpp"

#ifdef _WIN32
#define _WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <winsock2.h>

#pragma comment(lib, "Ws2_32.lib")

#else
#include <fcntl.h>
#include <unistd.h>
#endif

#include <system_error>
#include <utility>

#ifdef _WIN32
#pragma warning(disable : 4267)
#endif

Socket::~Socket() {
#ifdef _WIN32
    if (m_fd != INVALID_SOCKET) {
        closesocket(m_fd);
    }
#else
    if (m_fd != -1) {
        close(m_fd);
    }
#endif
}

Socket::Socket(Socket&& rhs) { std::swap(m_fd, rhs.m_fd); }

Socket& Socket::operator=(Socket&& rhs) {
    std::swap(m_fd, rhs.m_fd);

    return *this;
}

bool Socket::Read(char* buf, size_t length) {
    size_t pos = 0;
    while (pos < length) {
        int count = recv(m_fd, buf + pos, length - pos, 0);
        if (count == 0 || (count == -1 && errno != EAGAIN)) {
            return false;
        }
        pos += count;
    }

    return true;
}

size_t Socket::Write(std::string_view data) {
    return send(m_fd, data.data(), data.length(), 0);
}

bool Socket::WriteBlocking(std::string_view data) {
    size_t pos = 0;
    while (pos < data.length()) {
        int count = send(m_fd, data.data() + pos, data.length() - pos, 0);
        if (count == -1) {
            return false;
        }
        pos += count;
    }

    return true;
}

void Socket::SetBlocking(bool blocking) {
#ifdef _WIN32
    u_long nbenable = !blocking;
    if (ioctlsocket(m_fd, FIONBIO, &nbenable) != 0) {
        throw std::system_error(errno, std::system_category(), "Socket");
    }
#else
    int flags = fcntl(m_fd, F_GETFL, 0);
    if (flags == -1) {
        throw std::system_error(errno, std::system_category(), "Socket");
    }

    if (blocking) {
        fcntl(m_fd, F_SETFL, flags & ~O_NONBLOCK);
    } else {
        fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);
    }
#endif
}

#ifdef _WIN32
struct SocketInitializer {
    SocketInitializer() {
        WSADATA init;
        WSAStartup(MAKEWORD(2, 2), &init);
    }

    ~SocketInitializer() { WSACleanup(); }
};

SocketInitializer globalInitializer;
#endif
