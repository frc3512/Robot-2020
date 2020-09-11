// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "livegrapher/SocketSelector.hpp"

#include <system_error>

SocketSelector::SocketSelector() {
    FD_ZERO(&m_readFds);
    FD_ZERO(&m_writeFds);
    FD_ZERO(&m_errorFds);
    FD_ZERO(&m_selectReadFds);
    FD_ZERO(&m_selectWriteFds);
    FD_ZERO(&m_selectErrorFds);

    Add(m_pipe, kRead);
}

void SocketSelector::Add(const Socket& socket, int selectFlags) {
    Add(socket.m_fd, selectFlags);
}

void SocketSelector::Remove(const Socket& socket, int selectFlags) {
    Remove(socket.m_fd, selectFlags);
}

void SocketSelector::Add(const Pipe& pipe, int selectFlags) {
    if (selectFlags & kRead) {
        Add(pipe.m_fds[0], kRead);
    }
    if (selectFlags & kWrite) {
        Add(pipe.m_fds[1], kWrite);
    }
}

void SocketSelector::Remove(const Pipe& pipe, int selectFlags) {
    if (selectFlags & kRead) {
        Remove(pipe.m_fds[0], kRead);
    }
    if (selectFlags & kWrite) {
        Remove(pipe.m_fds[1], kWrite);
    }
}

bool SocketSelector::Select() {
    m_selectReadFds = m_readFds;
    m_selectWriteFds = m_writeFds;
    m_selectErrorFds = m_errorFds;

    int ret = select(m_maxFd + 1, &m_selectReadFds, &m_selectWriteFds,
                     &m_selectErrorFds, nullptr);

    // If the select() was cancelled via IPC, clear the IPC channel
    if (IsReadReady(m_pipe)) {
        char ipc;
        m_pipe.Read(&ipc, 1);
    }

    if (ret == -1) {
        throw std::system_error(errno, std::system_category(),
                                "SocketSelector");
    }

    return ret > 0;
}

bool SocketSelector::IsReadReady(const Socket& socket) {
    return FD_ISSET(socket.m_fd, &m_selectReadFds);
}

bool SocketSelector::IsWriteReady(const Socket& socket) {
    return FD_ISSET(socket.m_fd, &m_selectWriteFds);
}

bool SocketSelector::IsErrorReady(const Socket& socket) {
    return FD_ISSET(socket.m_fd, &m_selectErrorFds);
}

bool SocketSelector::IsReadReady(const Pipe& pipe) {
    return FD_ISSET(pipe.m_fds[0], &m_selectReadFds);
}

bool SocketSelector::IsWriteReady(const Pipe& pipe) {
    return FD_ISSET(pipe.m_fds[1], &m_selectWriteFds);
}

void SocketSelector::Cancel() { m_pipe.Write("x"); }

void SocketSelector::Add(int fd, int selectFlags) {
    if (fd > m_maxFd) {
        m_maxFd = fd;
    }

    if (selectFlags & kRead) {
        FD_SET(fd, &m_readFds);
    } else if (selectFlags & kWrite) {
        FD_SET(fd, &m_writeFds);
    } else if (selectFlags & kError) {
        FD_SET(fd, &m_errorFds);
    }
}

void SocketSelector::Remove(int fd, int selectFlags) {
    if (selectFlags & kRead) {
        FD_CLR(fd, &m_readFds);
    } else if (selectFlags & kWrite) {
        FD_CLR(fd, &m_writeFds);
    } else if (selectFlags & kError) {
        FD_CLR(fd, &m_errorFds);
    }
}
