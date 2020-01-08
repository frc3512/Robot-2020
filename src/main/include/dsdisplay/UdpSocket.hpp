// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdint.h>
#include <unistd.h>

namespace frc3512 {

class Packet;

/**
 * Specialized socket using the UDP protocol
 */
class UdpSocket {
public:
    enum Status {
        Done,          // The socket has sent / received the data
        NotReady,      // The socket is not ready to send / receive data yet
        Disconnected,  // The TCP socket has been disconnected
        Error          // An unexpected error happened
    };

    // Maximum number of bytes that can be sent in a single UDP datagram
    static constexpr size_t kMaxDatagramSize = 65507;

    // Special value that tells the system to pick any available port
    static constexpr uint32_t kAnyPort = 0;

    UdpSocket() = default;
    ~UdpSocket();

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket) = delete;

    /**
     * Bind the socket to a specific port
     *
     * Binding the socket to a port is necessary for being able to receive data
     * on that port. You can use the special value UdpSocket::kAnyPort to tell
     * the system to automatically pick an available port.
     *
     * @param port Port to bind the socket to
     * @return Status code
     *
     * @see unbind
     */
    Status bind(uint16_t port);

    /**
     * Unbind the socket from the local port to which it is bound
     *
     * The port that the socket was previously using is immediately available
     * after this function is called. If the socket is not bound to a port, this
     * function has no effect.
     *
     * @see bind
     */
    void unbind();

    /**
     * Send raw data to a remote peer
     *
     * Make sure that \a size is not greater than UdpSocket::MaxDatagramSize,
     * otherwise this function will fail and no data will be sent.
     *
     * @param data          Pointer to the sequence of bytes to send
     * @param size          Number of bytes to send
     * @param remoteAddress Address of the receiver
     * @param remotePort    Port of the receiver to send the data to
     *
     * @return Status code
     *
     * @see receive
     */
    Status send(const void* data, size_t size, uint32_t remoteAddress,
                uint16_t remotePort);

    /**
     * Receive raw data from a remote peer
     *
     * In blocking mode, this function will wait until some bytes are actually
     * received. Be careful to use a buffer which is large enough for the data
     * that you intend to receive, if it is too small then an error will be
     * returned and *all* the data will be lost.
     *
     * @param data          Pointer to the array to fill with the received bytes
     * @param size          Maximum number of bytes that can be received
     * @param received      This variable is filled with the actual number of
     * bytes received
     * @param remoteAddress Address of the peer that sent the data
     * @param remotePort    Port of the peer that sent the data
     *
     * @return Status code
     *
     * @see send
     */
    Status receive(void* data, size_t size, size_t& received,
                   uint32_t& remoteAddress, uint16_t& remotePort);

    /**
     * Send a formatted packet of data to a remote peer
     *
     * Make sure that the packet size is not greater than
     * UdpSocket::MaxDatagramSize, otherwise this function will fail and no
     * data will be sent.
     *
     * @param packet        Packet to send
     * @param remoteAddress Address of the receiver
     * @param remotePort    Port of the receiver to send the data to
     * @return Status code
     *
     * @see receive
     */
    Status send(Packet& packet, uint32_t remoteAddress, uint16_t remotePort);

    /**
     * Set the blocking state of the socket
     *
     * In blocking mode, calls will not return until they have completed their
     * task. For example, a call to Receive in blocking mode won't return until
     * some data was actually received. In non-blocking mode, calls will always
     * return immediately, using the return code to signal whether there was
     * data available or not. By default, all sockets are blocking.
     *
     * @param blocking 'true for blocking or 'false' for non-blocking
     */
    void setBlocking(bool blocking);

    /**
     * Tell whether the socket is in blocking or non-blocking mode
     *
     * @return 'true' if the socket is blocking, 'false' otherwise
     */
    bool isBlocking() const;

private:
    int m_socket = -1;        // Socket descriptor
    bool m_isBlocking{true};  // Current blocking mode of the socket

    // Create the internal representation of the socket
    void create();

    // Create the internal representation of the socket from a socket handle
    void create(int handle);

    // Close the socket gracefully
    void close();

    /* Create an internal sockaddr_in address
     * Params:
     *     address Target address
     *     port    Target port
     * Returns sockaddr_in ready to be used by socket functions
     */
    static sockaddr_in createAddress(uint32_t address, uint16_t port);

    // Returns status corresponding to the last socket error
    static Status getErrorStatus();
};

}  // namespace frc3512
