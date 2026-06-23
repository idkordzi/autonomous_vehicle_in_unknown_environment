#include "socket_udp.hpp"


namespace NAVIGATION_CORE {
 
SocketUDP::SocketUDP(bool reuseaddress, bool blocking) {
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("SocketUDP creation failed");
        exit(EXIT_FAILURE);
    }

    #ifndef _WIN32
    // Windows does not support FD_CLOEXEC
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    #endif
    if (reuseaddress) {
        set_reuseaddress();
    }
    if (blocking) {
        set_blocking(true);
    }
}

SocketUDP::~SocketUDP() {
    if (fd != -1) {
        ::close(fd);
        fd = -1;
    }
}

bool SocketUDP::bind(const char *address, uint16_t port) {
    struct sockaddr_in server_addr{};
    make_sockaddr(address, port, server_addr);

    if (::bind(fd, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) != 0) {
        perror("SocketUDP Bind failed");
    #ifdef _WIN32
        closesocket(fd);
    #else
        close(fd);
    #endif
        return false;
    }
    return true;
}

bool SocketUDP::set_reuseaddress() {
    int one = 1;
    return (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) != -1);
}

bool SocketUDP::set_blocking(bool blocking) {
    int fcntl_ret;
#ifdef _WIN32
    u_long mode = blocking ? 0 : 1;
    fcntl_ret = ioctlsocket(fd, FIONBIO, reinterpret_cast<u_long FAR *>(&mode));
#else
    if (blocking) {
        fcntl_ret = fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
    } else {
        fcntl_ret = fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    }
#endif
    return fcntl_ret != -1;
}

ssize_t SocketUDP::sendto(const void *buf, size_t size, const char *address, uint16_t port) {
    struct sockaddr_in sockaddr_out{};
    make_sockaddr(address, port, sockaddr_out);
    return ::sendto(fd, buf, size, 0, reinterpret_cast<sockaddr *>(&sockaddr_out), sizeof(sockaddr_out));
}

ssize_t SocketUDP::recv(void *buf, size_t size, uint32_t timeout_ms) {
    if (!pollin(timeout_ms)) {
        return -1;
    }
    socklen_t len = sizeof(in_addr);
    return ::recvfrom(fd, buf, size, MSG_DONTWAIT, reinterpret_cast<sockaddr *>(&in_addr), &len);
}

void SocketUDP::get_client_address(const char *&ip_addr, uint16_t &port) {
    ip_addr = inet_ntoa(in_addr.sin_addr);
    port = ntohs(in_addr.sin_port);
}

bool SocketUDP::pollin(uint32_t timeout_ms) {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;

    if (select(fd + 1, &fds, nullptr, nullptr, &tv) != 1) {
        return false;
    }
    return true;
}

void SocketUDP::make_sockaddr(const char *address, uint16_t port, struct sockaddr_in &sockaddr) {
    sockaddr = {};

    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(address);
    sockaddr.sin_port = htons(port);
#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
}

}  // namespace NAVIGATION_CORE
