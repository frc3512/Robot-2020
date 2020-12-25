// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#ifdef _WIN32
#define _WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <winsock2.h>

#else
#include <sys/select.h>
#endif

#include "livegrapher/Pipe.hpp"
#include "livegrapher/Socket.hpp"

class SocketSelector {
public:
    enum Select { kRead = 1, kWrite = 2, kError = 4 };

    SocketSelector();

    /**
     * Add socket to the selector.
     *
     * @param socket      The socket to add.
     * @param selectFlags A bitfield representing whether to select on read,
     *                    write, and/or error.
     */
    void Add(const Socket& socket, int selectFlags);

    /**
     * Remove socket from the selector.
     *
     * @param socket      The socket to remove.
     * @param selectFlags A bitfield representing whether to stop selecting on
     *                    read, write, and/or error.
     */
    void Remove(const Socket& socket, int selectFlags);

    /**
     * Add pipe to the selector.
     *
     * When selecting on read, the read end of the pipe will be used. When
     * selecting on write, the write end of the pipe will be used.
     *
     * @param pipe        The pipe to add.
     * @param selectFlags A bitfield representing whether to select on read
     *                    and/or write.
     */
    void Add(const Pipe& pipe, int selectFlags);

    /**
     * Remove pipe from the selector.
     *
     * When selecting on read, the read end of the pipe will be used. When
     * selecting on write, the write end of the pipe will be used.
     *
     * @param pipe        The pipe to remove.
     * @param selectFlags A bitfield representing whether to stop selecting on
     *                    read and/or write.
     */
    void Remove(const Pipe& pipe, int selectFlags);

    /**
     * Selects on all the registered sockets.
     *
     * @return True if a socket is ready.
     */
    bool Select();

    /**
     * Returns true if socket is ready to read.
     *
     * @param socket Socket to check.
     */
    bool IsReadReady(const Socket& socket);

    /**
     * Returns true if socket is ready to write.
     *
     * @param socket Socket to check.
     */
    bool IsWriteReady(const Socket& socket);

    /**
     * Returns true if socket has encountered an error.
     *
     * @param socket Socket to check.
     */
    bool IsErrorReady(const Socket& socket);

    /**
     * Returns true if read end of pipe is ready.
     *
     * @param pipe Pipe to check.
     */
    bool IsReadReady(const Pipe& pipe);

    /**
     * Returns true if write end of pipe is ready.
     *
     * @param pipe Pipe to check.
     */
    bool IsWriteReady(const Pipe& pipe);

    /**
     * Cancel a blocking Select() call, causing it to return immediately.
     */
    void Cancel();

private:
    fd_set m_readFds;
    fd_set m_writeFds;
    fd_set m_errorFds;
    fd_set m_selectReadFds;
    fd_set m_selectWriteFds;
    fd_set m_selectErrorFds;
#ifdef _WIN32
    SOCKET m_maxFd = 0;
#else
    int m_maxFd = 0;
#endif

    Pipe m_pipe;

    /**
     * Add socket file descriptor to the selector.
     *
     * @param fd          The socket file descriptor to remove.
     * @param selectFlags A bitfield representing whether to select on read,
     *                    write, and/or error.
     */
#ifdef _WIN32
    void Add(SOCKET fd, int selectFlags);
#else
    void Add(int fd, int selectFlags);
#endif

    /**
     * Remove socket file descriptor from the selector.
     *
     * @param fd          The socket file descriptor to remove.
     * @param selectFlags A bitfield representing whether to stop selecting on
     *                    read, write, and/or error.
     */
#ifdef _WIN32
    void Remove(SOCKET fd, int selectFlags);
#else
    void Remove(int fd, int selectFlags);
#endif
};
