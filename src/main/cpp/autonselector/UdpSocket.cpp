// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "autonselector/UdpSocket.hpp"

#include <fcntl.h>
#include <sys/socket.h>

#include <cstring>

#include <wpi/raw_ostream.h>

#include "autonselector/Packet.hpp"

using namespace frc3512;

UdpSocket::~UdpSocket() { close(); }

UdpSocket::Status UdpSocket::bind(uint16_t port) {
    // Create the internal socket if it doesn't exist
    create();

    // Bind the socket
    sockaddr_in address = UdpSocket::createAddress(INADDR_ANY, port);
    if (::bind(m_socket, reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) == -1) {
        wpi::errs() << "Failed to bind socket to port " << port << "\n";
        return Error;
    }

    return Done;
}

void UdpSocket::unbind() {
    // Simply close the socket
    close();
}

UdpSocket::Status UdpSocket::send(const void* data, size_t size,
                                  uint32_t remoteAddress, uint16_t remotePort) {
    // Create the internal socket if it doesn't exist
    create();

    // Make sure that all the data will fit in one datagram
    if (size > kMaxDatagramSize) {
        wpi::errs() << "Cannot send data over the network (the number of bytes "
                       "to send is greater than UdpSocket::kMaxDatagramSize)\n";
        return Error;
    }

    // Build the target address
    sockaddr_in address = UdpSocket::createAddress(remoteAddress, remotePort);

    // Send the data (unlike TCP, all the data is always sent in one call)
    int sent = sendto(m_socket, static_cast<char*>(const_cast<void*>(data)),
                      static_cast<int>(size), 0,
                      reinterpret_cast<sockaddr*>(&address), sizeof(address));

    // Check for errors
    if (sent < 0) {
        return UdpSocket::getErrorStatus();
    }

    return Done;
}

UdpSocket::Status UdpSocket::receive(void* data, size_t size, size_t& received,
                                     uint32_t& remoteAddress,
                                     uint16_t& remotePort) {
    // First clear the variables to fill
    received = 0;
    remoteAddress = 0;
    remotePort = 0;

    // Check the destination buffer
    if (!data) {
        wpi::errs() << "Cannot receive data from the network (the destination "
                       "buffer is invalid)\n";
        return Error;
    }

    // Data that will be filled with the other computer's address
    sockaddr_in address = UdpSocket::createAddress(INADDR_ANY, 0);

    // Receive a chunk of bytes
    socklen_t addressSize = sizeof(address);
    socklen_t temp = addressSize;
    int sizeReceived =
        recvfrom(m_socket, static_cast<char*>(const_cast<void*>(data)),
                 static_cast<int>(size), 0,
                 reinterpret_cast<sockaddr*>(&address), &temp);

    // Check for errors
    if (sizeReceived < 0) {
        return UdpSocket::getErrorStatus();
    }

    // Fill the sender informations
    received = static_cast<size_t>(sizeReceived);
    remoteAddress = ntohl(address.sin_addr.s_addr);
    remotePort = ntohs(address.sin_port);

    return Done;
}

UdpSocket::Status UdpSocket::send(Packet& packet, uint32_t remoteAddress,
                                  uint16_t remotePort) {
    /* UDP is a datagram-oriented protocol (as opposed to TCP which is a stream
     * protocol). Sending one datagram is almost safe: it may be lost but if
     * it's received, then its data is guaranteed to be ok. However, splitting
     * a packet into multiple datagrams would be highly unreliable, since
     * datagrams may be reordered, dropped or mixed between different sources.
     * That's why SFML imposes a limit on packet size so that they can be sent
     * in a single datagram. This also removes the overhead associated to
     * packets -- there's no size to send in addition to the packet's data.
     */

    // Get the data to send from the packet
    size_t size = packet.getDataSize();
    const void* data = packet.getData();

    // Send it
    return send(data, size, remoteAddress, remotePort);
}

void UdpSocket::setBlocking(bool blocking) {
    int status = fcntl(m_socket, F_GETFL);
    if (blocking) {
        fcntl(m_socket, F_SETFL, status & ~O_NONBLOCK);
    } else {
        fcntl(m_socket, F_SETFL, status | O_NONBLOCK);
    }

    m_isBlocking = blocking;
}

bool UdpSocket::isBlocking() const { return m_isBlocking; }

void UdpSocket::create() {
    // Don't create the socket if it already exists
    if (m_socket == -1) {
        int handle = socket(PF_INET, SOCK_DGRAM, 0);
        create(handle);
    }
}

void UdpSocket::create(int handle) {
    // Don't create the socket if it already exists
    if (m_socket == -1) {
        // Assign the new handle
        m_socket = handle;

        // Set the current blocking state
        setBlocking(m_isBlocking);

        // Enable broadcast by default for UDP sockets
        int yes = 1;
        if (setsockopt(m_socket, SOL_SOCKET, SO_BROADCAST,
                       reinterpret_cast<char*>(&yes), sizeof(yes)) == -1) {
            wpi::errs() << "Failed to enable broadcast on UDP socket\n";
        }
    }
}

void UdpSocket::close() {
    // Close the socket
    if (m_socket != -1) {
        ::close(m_socket);
        m_socket = -1;
    }
}

sockaddr_in UdpSocket::createAddress(uint32_t address, uint16_t port) {
    sockaddr_in addr;
    std::memset(addr.sin_zero, 0, sizeof(addr.sin_zero));
    addr.sin_addr.s_addr = htonl(address);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    return addr;
}

UdpSocket::Status UdpSocket::getErrorStatus() {
    // The followings are sometimes equal to EWOULDBLOCK,
    // so we have to make a special case for them in order
    // to avoid having double values in the switch case
    if ((errno == EAGAIN) || (errno == EINPROGRESS)) {
        return UdpSocket::NotReady;
    }

    switch (errno) {
        case EWOULDBLOCK:
            return UdpSocket::NotReady;
        case ECONNABORTED:
            return UdpSocket::Disconnected;
        case ECONNRESET:
            return UdpSocket::Disconnected;
        case ETIMEDOUT:
            return UdpSocket::Disconnected;
        case ENETRESET:
            return UdpSocket::Disconnected;
        case ENOTCONN:
            return UdpSocket::Disconnected;
        default:
            return UdpSocket::Error;
    }
}
