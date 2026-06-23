#include <fcntl.h>
#include <unistd.h>

#ifdef _WIN32

#include <winsock2.h>
#include <Ws2tcpip.h>

#else

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/select.h>

#endif

#include <cstdio>
#include <cstdlib>
#include <cstring>


namespace NAVIGATION_CORE {

struct servo_packet_16 {
    uint16_t magic {18458};
    uint16_t frame_rate {0};
    uint32_t frame_count {0};
    uint16_t pwm[16] = {0};
};
 
class SocketUDP {

public:

    SocketUDP(bool reuseaddress, bool blocking);
    ~SocketUDP();

    bool bind(const char *address, uint16_t port);

    bool set_reuseaddress();
    bool set_blocking(bool blocking);

    ssize_t sendto(const void *buf, size_t size, const char *address, uint16_t port);
    ssize_t recv(void *buf, size_t size, uint32_t timeout_ms);

    void get_client_address(const char *&ip_addr, uint16_t &port);

private:

    struct sockaddr_in in_addr{};
    int fd = -1;

    bool pollin(uint32_t timeout_ms);
    void make_sockaddr(const char *address, uint16_t port, struct sockaddr_in &sockaddr);
};

}  // namespace NAVIGATION_CORE
