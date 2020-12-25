// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#ifdef _WIN32

#include <cstring>

#include "livegrapher/Win32PipeImpl.hpp"

/**
 * Emulates pipe(3).
 *
 * @param fds Array of file descriptors for the pipe where fds[0] is the pipe's
 *            read end and fds[1] is the pipe's write end.
 * @return 0 on success or -1 on failure.
 */
int pipe(SOCKET fds[2]) {
    if (fds == nullptr) {
        WSASetLastError(WSAEINVAL);
        return SOCKET_ERROR;
    }

    SOCKET listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener == INVALID_SOCKET) {
        return SOCKET_ERROR;
    }

    struct sockaddr_in pin;
    std::memset(&pin, 0, sizeof(pin));
    pin.sin_family = AF_INET;
    pin.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    pin.sin_port = 0;

    fds[0] = INVALID_SOCKET;
    fds[1] = INVALID_SOCKET;

    do {
        int reuse = 1;
        if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR,
                       reinterpret_cast<char*>(&reuse), sizeof(reuse)) == -1) {
            break;
        }

        if (bind(listener, reinterpret_cast<struct sockaddr*>(&pin),
                 sizeof(struct sockaddr_in)) == SOCKET_ERROR) {
            break;
        }

        int addrlen = sizeof(struct sockaddr_in);
        if (getsockname(listener, reinterpret_cast<struct sockaddr*>(&pin),
                        &addrlen) == SOCKET_ERROR) {
            break;
        }

        if (listen(listener, 1) == SOCKET_ERROR) {
            break;
        }

        fds[0] = WSASocketW(AF_INET, SOCK_STREAM, 0, nullptr, 0, 0);
        if (fds[0] == INVALID_SOCKET) {
            break;
        }

        if (connect(fds[0], reinterpret_cast<struct sockaddr*>(&pin),
                    sizeof(struct sockaddr)) == SOCKET_ERROR) {
            break;
        }

        fds[1] = accept(listener, nullptr, nullptr);
        if (fds[1] == INVALID_SOCKET) {
            break;
        }

        closesocket(listener);

        return 0;
    } while (0);

    int e = WSAGetLastError();
    closesocket(listener);
    closesocket(fds[0]);
    closesocket(fds[1]);
    WSASetLastError(e);

    return SOCKET_ERROR;
}
#endif
