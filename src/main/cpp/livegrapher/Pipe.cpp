// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "livegrapher/Pipe.hpp"

#include <unistd.h>

#include <system_error>
#include <utility>

Pipe::Pipe() {
    if (pipe(m_fds) == -1) {
        throw std::system_error(errno, std::system_category(), "Pipe");
    }
}

Pipe::~Pipe() {
    if (m_fds[0] != -1) {
        close(m_fds[0]);
    }
    if (m_fds[1] != -1) {
        close(m_fds[1]);
    }
}

Pipe::Pipe(Pipe&& rhs) {
    std::swap(m_fds[0], rhs.m_fds[0]);
    std::swap(m_fds[1], rhs.m_fds[1]);
}

Pipe& Pipe::operator=(Pipe&& rhs) {
    std::swap(m_fds[0], rhs.m_fds[0]);
    std::swap(m_fds[1], rhs.m_fds[1]);

    return *this;
}

bool Pipe::Read(char* buf, size_t length) {
    size_t pos = 0;
    while (pos < length) {
        int count = read(m_fds[0], buf + pos, length - pos);
        if (count == 0 || (count == -1 && errno != EAGAIN)) {
            return false;
        }
        pos += count;
    }

    return true;
}

size_t Pipe::Write(std::string_view data) {
    return write(m_fds[1], data.data(), data.length());
}

bool Pipe::WriteBlocking(std::string_view data) {
    size_t pos = 0;
    while (pos < data.length()) {
        int count = write(m_fds[1], data.data() + pos, data.length() - pos);
        if (count == -1) {
            return false;
        }
        pos += count;
    }

    return true;
}
