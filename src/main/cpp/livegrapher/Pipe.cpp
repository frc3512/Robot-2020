// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "livegrapher/Pipe.hpp"

#include <system_error>
#include <utility>

#include "livegrapher/Win32PipeImpl.hpp"

#ifdef _WIN32
#pragma warning(disable : 4267)
#endif

Pipe::Pipe() {
    if (pipe(m_fds) == -1) {
        throw std::system_error(errno, std::system_category(), "Pipe");
    }
}

Pipe::~Pipe() {
#ifdef _WIN32
    if (m_fds[0] != INVALID_SOCKET) {
        closesocket(m_fds[0]);
    }
    if (m_fds[1] != INVALID_SOCKET) {
        closesocket(m_fds[1]);
    }
#else
    if (m_fds[0] != -1) {
        close(m_fds[0]);
    }
    if (m_fds[1] != -1) {
        close(m_fds[1]);
    }
#endif
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
#ifdef _WIN32
        int count = recv(m_fds[0], buf + pos, length - pos, 0);
#else
        int count = read(m_fds[0], buf + pos, length - pos);
#endif
        if (count == 0 || (count == -1 && errno != EAGAIN)) {
            return false;
        }
        pos += count;
    }

    return true;
}

size_t Pipe::Write(std::string_view data) {
#ifdef _WIN32
    return send(m_fds[1], data.data(), data.length(), 0);
#else
    return write(m_fds[1], data.data(), data.length());
#endif
}

bool Pipe::WriteBlocking(std::string_view data) {
    size_t pos = 0;
    while (pos < data.length()) {
#ifdef _WIN32
        int count = send(m_fds[1], data.data() + pos, data.length() - pos, 0);
#else
        int count = write(m_fds[1], data.data() + pos, data.length() - pos);
#endif
        if (count == -1) {
            return false;
        }
        pos += count;
    }

    return true;
}
