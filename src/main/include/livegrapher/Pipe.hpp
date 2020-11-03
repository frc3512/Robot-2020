// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <cerrno>
#include <string_view>

class Pipe {
public:
    Pipe();
    ~Pipe();

    Pipe(Pipe&& rhs);
    Pipe& operator=(Pipe&& rhs);

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
            int count = read(m_fds[0], buf.data() + pos, buf.size() - pos);
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
     * @param buf    The destination for the bytes read.
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

private:
    friend class SocketSelector;

    int m_fds[2];
};
