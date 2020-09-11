// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <sys/socket.h>

#include <array>
#include <cerrno>
#include <string_view>

class Socket {
public:
    Socket() = default;
    explicit Socket(int fd);
    ~Socket();

    Socket(Socket&& rhs);
    Socket& operator=(Socket&& rhs);

    /**
     * Read the given amount of bytes from the socket.
     *
     * This blocks until the buffer is full.
     *
     * @param buf The destination for the bytes read.
     * @return True if the read succeeded.
     */
    template <size_t N>
    bool Read(std::array<char, N>& buf) {
        size_t pos = 0;
        while (pos < buf.size()) {
            int count = recv(m_fd, buf.data() + pos, buf.size() - pos, 0);
            if (count == 0 || (count == -1 && errno != EAGAIN)) {
                return false;
            }
            pos += count;
        }

        return true;
    }

    /**
     * Read the given amount of bytes from the socket.
     *
     * This blocks until the buffer is full.
     *
     * @param buffer The destination for the bytes read.
     * @param length The number of bytes to read.
     * @return True if the read succeeded.
     */
    bool Read(char* buf, size_t length);

    /**
     * Send a string of data.
     *
     * @param data The string of data to send.
     * @return The number of bytes actually written, or -1 on error.
     */
    size_t Write(std::string_view data);

    /**
     * Send a string of data.
     *
     * @param data The string of data to send.
     * @return True if the write succeeded.
     */
    bool WriteBlocking(std::string_view data);

    /**
     * Set whether socket blocks on reads and writes.
     *
     * @param blocking True if socket should block.
     */
    void SetBlocking(bool blocking);

protected:
    friend class SocketSelector;

    int m_fd = -1;
};
