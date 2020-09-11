// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "livegrapher/Socket.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <system_error>
#include <utility>

Socket::Socket(int fd) : m_fd(fd) {}

Socket::~Socket() {
    if (m_fd != -1) {
        close(m_fd);
    }
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
    int flags = fcntl(m_fd, F_GETFL, 0);
    if (flags == -1) {
        throw std::system_error(errno, std::system_category(), "Socket");
    }

    if (blocking) {
        fcntl(m_fd, F_SETFL, flags & ~O_NONBLOCK);
    } else {
        fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);
    }
}
