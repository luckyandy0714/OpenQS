#pragma once
#include <stdio.h>

#define LOG_MAX_LENGTH 50

enum Protocol_t
{
    TCP = 0,
    UDP,
};
enum Role_t
{
    SERVER = 0,
    CLIENT,
};

class Socket
{
private:
    char *TAG;
    Protocol_t protocol;
    Role_t role;
    int socket_handl;
    int tcp_connection_handl;
    void *addr_ptr;
    int keepAlive;
    int keepIdle;
    int keepInterval;
    int keepCount;

public:
    Socket(Protocol_t protocol, Role_t role);
    ~Socket();

    bool server_create(uint32_t serverIP, uint16_t serverPort);
    bool server_create(const char *serverIP, uint16_t serverPort);
    bool server_create(uint16_t serverPort);

    bool client_connect(uint32_t hostIP, uint16_t hostPort, bool retry);
    bool client_connect(const char *hostIP, uint16_t hostPort, bool retry);
    void close_socket();

    bool send(const char *src, size_t len);
    bool recv(char *dst, size_t *len);

    uint32_t get_server_addr();
};

class UDP_MultiCast
{
protected:
    int socket_handl;

private:
    char *TAG;
    void *addr_ptr;

public:
    UDP_MultiCast();
    ~UDP_MultiCast();
    bool server_create(uint32_t multiCastIP, uint16_t multiCastPort);
    bool server_create(const char *multiCastIP, uint16_t multiCastPort);
    void close_socket();

    bool send(const char *src, size_t len, bool byap = false);
    bool recv(char *dst, size_t *len, void *addr, uint8_t time_out = 0);
};

typedef struct
{
    uint16_t id;
    uint16_t flags;
    uint16_t questions;
    uint16_t answers;
    uint16_t auth_rr;
    uint16_t add_rr;
} mdns_header_t;

class mDNS : public UDP_MultiCast
{
private:
    char *TAG;
    uint32_t mask;
    void get_ip_info(void *ap_addr_info, void *sta_addr_info);
    size_t strToNameFormat(const char *src, size_t len, char *dst);

public:
    mDNS();
    ~mDNS();
    bool init();
    bool ask_if_same_devic_name();
    void listen();
};