#include "CustomSocket.h"
#include <cstring>

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

#include <sys/socket.h>
#include <netdb.h>

#include "esp_log.h"

#define SOCKET_RX_BUFFER_SIZE 1024
#define SOCKET_MAX_NUMBER 10

#define SOCKET_CONNECT_RETRY_MAX_NUM 100

#define KEEPALIVE_IDLE 5
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT 5

#define INVALID_SOCKET (-1)

static uint16_t Socket_Counter;

Socket::Socket(Protocol_t protocol, Role_t role)
{
    if (Socket_Counter > SOCKET_MAX_NUMBER)
        return;
    Socket_Counter++;
    this->protocol = protocol;
    this->role = role;
    const char *protocol_str = (protocol == TCP) ? "TCP" : "UDP";
    const char *role_str = (role == SERVER) ? "SERVER" : "CLIENT";
    char buffer[24];
    int str_len = sprintf(buffer, "%s_%s_%s_%d", "SOCKET", protocol_str, role_str, Socket_Counter);
    TAG = (char *)calloc(1, str_len + 1);
    if (TAG == NULL)
        return;
    memcpy(TAG, buffer, str_len + 1);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    err = esp_netif_init();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "esp_netif_init failed (%s)", esp_err_to_name(err));
    err = esp_event_loop_create_default();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "esp_event_loop_create_default failed (%s)", esp_err_to_name(err));

    socket_handl = INVALID_SOCKET;
    addr_ptr = NULL;

    keepAlive = 1;
    keepIdle = KEEPALIVE_IDLE;
    keepInterval = KEEPALIVE_INTERVAL;
    keepCount = KEEPALIVE_COUNT;
}

Socket::~Socket()
{
    close_socket();
    free(TAG);
    if (addr_ptr)
        free(addr_ptr);
}

bool Socket::server_create(uint32_t serverIP, uint16_t serverPort)
{
    if (role != SERVER)
        return false;
    if (socket_handl != INVALID_SOCKET)
        return true;
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(serverIP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(serverPort);

    socket_handl = socket(AF_INET, (protocol == TCP) ? SOCK_STREAM : SOCK_DGRAM, IPPROTO_IP);
    if (socket_handl < 0)
    {
        ESP_LOGE(TAG, "Unable to create server socket: errno %d", errno);
        return false;
    }
    if (protocol == TCP)
    {
        int opt = 1;
        setsockopt(socket_handl, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    }
    else
    {
        struct timeval timeout = {
            .tv_sec = 10,
            .tv_usec = 0,
        };
        setsockopt(socket_handl, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(socket_handl, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket server unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", AF_INET);
        close_socket();
        return false;
    }
    char addr_str[16] = {};
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
    ESP_LOGI(TAG, "Socket server bound: %s:%d", addr_str, serverPort);
    if (protocol == TCP)
    {
        err = listen(socket_handl, 1);
        if (err != 0)
        {
            ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
            close_socket();
            return false;
        }
    }
    return true;
}
bool Socket::server_create(const char *serverIP, uint16_t serverPort)
{
    uint32_t ip;
    inet_pton(AF_INET, serverIP, &ip);
    return server_create(ip, serverPort);
}
bool Socket::server_create(uint16_t serverPort)
{
    return server_create(INADDR_ANY, serverPort);
}
bool Socket::client_connect(uint32_t hostIP, uint16_t hostPort, bool retry)
{
    if (role != CLIENT || socket_handl != INVALID_SOCKET)
        return false;
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(hostIP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(hostPort);

    socket_handl = socket(AF_INET, (protocol == TCP) ? SOCK_STREAM : SOCK_DGRAM, IPPROTO_IP);
    if (socket_handl < 0)
    {
        ESP_LOGE(TAG, "Unable to create client socket: errno %d", errno);
        return false;
    }
    if (!addr_ptr)
    {
        addr_ptr = malloc(sizeof(dest_addr));
        memcpy(addr_ptr, &dest_addr, sizeof(dest_addr));
    }
    char ip_buffer[16] = {};
    inet_ntop(AF_INET, &dest_addr.sin_addr.s_addr, ip_buffer, sizeof(ip_buffer));
    ESP_LOGI(TAG, "Socket client created, connecting to %s:%d", ip_buffer, hostPort);
    if (protocol == UDP)
    {
        struct timeval timeout = {
            .tv_sec = 10,
            .tv_usec = 0,
        };
        setsockopt(socket_handl, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        ESP_LOGI(TAG, "UDP client socket start, test send...");
        int err = sendto(socket_handl, "\0", 1, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            return false;
        }
    }
    else
    {
        int count = 0;
        do
        {
            int err = ::connect(socket_handl, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err != 0)
            {
                if (retry)
                {
                    if (++count > SOCKET_CONNECT_RETRY_MAX_NUM)
                    {
                        ESP_LOGE(TAG, "Too many retries");
                        break;
                    }
                    ESP_LOGI(TAG, "Retry connecting.... %d", count);
                    close(socket_handl);
                    socket_handl = socket(AF_INET, (protocol == TCP) ? SOCK_STREAM : SOCK_DGRAM, IPPROTO_IP);
                }
                else
                {
                    ESP_LOGE(TAG, "Socket client unable to connect: errno %d", errno);
                    close(socket_handl);
                    break;
                }
            }
            else
            {
                ESP_LOGI(TAG, "Socket client successfully connected");
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        } while (true);
    }
    return false;
}
bool Socket::client_connect(const char *hostIP, uint16_t hostPort, bool retry)
{
    uint32_t ip;
    inet_pton(AF_INET, hostIP, &ip);
    return client_connect(ip, hostPort, retry);
}
void Socket::close_socket()
{
    if (socket_handl != INVALID_SOCKET)
    {
        shutdown(socket_handl, 0);
        close(socket_handl);
        socket_handl = INVALID_SOCKET;
        if (addr_ptr)
            free(addr_ptr);
        addr_ptr = NULL;
    }
    ESP_LOGI(TAG, "Socket close");
}

bool Socket::send(const char *src, size_t len)
{
    if (socket_handl < 0)
    {
        ESP_LOGE(TAG, "socket_handler not available");
        return false;
    }
    if (protocol == TCP)
        ::send(socket_handl, src, len, 0);
    else
    {
        if (!addr_ptr)
            return false;
        ::sendto(socket_handl, src, len, 0, (struct sockaddr *)addr_ptr, sizeof(struct sockaddr));
    }
    return true;
}

bool Socket::recv(char *dst, size_t *len)
{
    ssize_t read_len;
    struct sockaddr_in source_addr;
    socklen_t addr_len = sizeof(source_addr);
    if (role == SERVER)
    {
        if (protocol == TCP && tcp_connection_handl == INVALID_SOCKET)
        {
            ESP_LOGI(TAG, "Socket listening");

            tcp_connection_handl = accept(socket_handl, (struct sockaddr *)&source_addr, &addr_len);
            if (tcp_connection_handl < 0)
            {
                ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
                close_socket();
                return false;
            }
            setsockopt(tcp_connection_handl, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
            setsockopt(tcp_connection_handl, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
            setsockopt(tcp_connection_handl, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
            setsockopt(tcp_connection_handl, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
            char addr_str[16] = {};
            inet_ntoa_r(source_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG, "Socket accepted ip address: %s:%d", addr_str, source_addr.sin_port);
        }

        if (protocol == TCP)
            read_len = ::recv(tcp_connection_handl, dst, *len, 0);
        else
            read_len = ::recvfrom(socket_handl, dst, *len, 0, (struct sockaddr *)&source_addr, &addr_len);
    }
    else
    {
        if (protocol == TCP)
            read_len = ::recv(socket_handl, dst, *len, 0);
        else
            read_len = ::recvfrom(socket_handl, dst, *len, 0, (struct sockaddr *)&source_addr, &addr_len);
    }
    if (read_len < 0)
    {
        if (errno != EAGAIN)
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
    }
    else if (read_len == 0)
    {
        ESP_LOGW(TAG, "Connection closed");
        if (role == SERVER && protocol == TCP)
        {
            shutdown(tcp_connection_handl, 0);
            close(tcp_connection_handl);
            tcp_connection_handl = INVALID_SOCKET;
        }
    }
    else
    {
        if (role == SERVER)
            if (!addr_ptr)
            {
                addr_ptr = malloc(sizeof(source_addr));
                memcpy(addr_ptr, &source_addr, sizeof(source_addr));
            }
        if (protocol == UDP)
            if (source_addr.sin_addr.s_addr != ((struct sockaddr_in *)addr_ptr)->sin_addr.s_addr ||
                source_addr.sin_port != ((struct sockaddr_in *)addr_ptr)->sin_port)
                return false;

        *len = read_len;
        if (*len < LOG_MAX_LENGTH)
        {
            dst[*len] = '\0';
            ESP_LOGI(TAG, "Receiver: %s", dst);
        }
        return true;
    }
    // ESP_LOGW(TAG, "Connection closed");
    // if (role == SERVER && protocol == TCP)
    // {
    //     shutdown(tcp_connection_handler, 0);
    //     close(tcp_connection_handler);
    //     tcp_connection_handler = INVALID_SOCKET;
    // }
    return false;
}
uint32_t Socket::get_server_addr()
{
    if (!addr_ptr)
        return 0;
    return ntohl(((struct sockaddr_in *)addr_ptr)->sin_addr.s_addr);
}

#define UDP_MULTICAST_TTL 1

UDP_MultiCast::UDP_MultiCast()
{
    const char *str = "UDP_MultiCast";
    TAG = (char *)calloc(1, strlen(str) + 1);
    if (TAG == NULL)
        return;
    memcpy(TAG, str, strlen(str) + 1);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    err = esp_netif_init();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "esp_netif_init failed (%s)", esp_err_to_name(err));
    err = esp_event_loop_create_default();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "esp_event_loop_create_default failed (%s)", esp_err_to_name(err));

    socket_handl = INVALID_SOCKET;
    addr_ptr = malloc(sizeof(struct sockaddr));
}

UDP_MultiCast::~UDP_MultiCast()
{
    close_socket();
    free(TAG);
    free(addr_ptr);
}

bool UDP_MultiCast::server_create(uint32_t multiCastIP, uint16_t multiCastPort)
{
    char ip_str[16] = {};
    inet_ntop(AF_INET, &multiCastIP, ip_str, sizeof(ip_str));

    struct sockaddr_in saddr = {};
    int err = 0;
    socket_handl = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (socket_handl < 0)
    {
        ESP_LOGE(TAG, "Failed to create socket. Error %d", errno);
        return false;
    }
    // Bind the socket to any address
    saddr.sin_family = PF_INET;
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons(multiCastPort);

    err = bind(socket_handl, (struct sockaddr *)&saddr, sizeof(saddr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Failed to bind socket. Error %d", errno);
        close_socket();
        return false;
    }
    saddr.sin_addr.s_addr = (multiCastIP);
    memcpy(addr_ptr, &saddr, sizeof(saddr));
    // Assign multicast TTL (set separately from normal interface TTL)
    uint8_t ttl = UDP_MULTICAST_TTL;
    setsockopt(socket_handl, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Failed to set IP_MULTICAST_TTL. Error %d", errno);
        close_socket();
        return false;
    }
    // Configure multicast address to listen to
    ESP_LOGI(TAG, "Configured Multicast address %s", ip_str);

    struct ip_mreq imreq = {};
    imreq.imr_multiaddr.s_addr = multiCastIP;
    imreq.imr_interface.s_addr = INADDR_ANY;

    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr)))
        ESP_LOGW(TAG, "Configured multicast address fail.");

    struct in_addr iaddr = {};
    err = setsockopt(socket_handl, IPPROTO_IP, IP_MULTICAST_IF, &iaddr, sizeof(struct in_addr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Failed to set IP_MULTICAST_IF. Error %d", errno);
        close_socket();
        return false;
    }
    err = setsockopt(socket_handl, IPPROTO_IP, IP_ADD_MEMBERSHIP, &imreq, sizeof(struct ip_mreq));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Failed to set IP_ADD_MEMBERSHIP. Error %d", errno);
        close_socket();
        return false;
    }
    ESP_LOGI(TAG, "Socket create success");
    return true;
}

bool UDP_MultiCast::server_create(const char *multiCastIP, uint16_t multiCastPort)
{
    uint32_t ip;
    inet_pton(AF_INET, multiCastIP, &ip);
    return server_create(ip, multiCastPort);
}

void UDP_MultiCast::close_socket()
{
    if (socket_handl != INVALID_SOCKET)
    {
        shutdown(socket_handl, 0);
        close(socket_handl);
        socket_handl = INVALID_SOCKET;
    }
    ESP_LOGI(TAG, "Socket close");
}

bool UDP_MultiCast::send(const char *src, size_t len, bool byap)
{
    if (socket_handl < 0)
    {
        ESP_LOGE(TAG, "socket_handler not available");
        return false;
    }
    if (!addr_ptr)
        return false;

    // esp_netif_t *netif = esp_netif_get_handle_from_ifkey(byap ? "WIFI_AP_DEF" : "WIFI_STA_DEF");
    // esp_netif_bind_socket(socket_handler, netif);

    ::sendto(socket_handl, src, len, 0, (struct sockaddr *)addr_ptr, sizeof(struct sockaddr));
    return true;
}

bool UDP_MultiCast::recv(char *dst, size_t *len, void *addr, uint8_t time_out)
{
    struct sockaddr_in *source_addr = (struct sockaddr_in *)addr;
    socklen_t addr_len = sizeof(*source_addr);

    if (time_out != 0)
    {
        struct timeval timeout_val = {
            .tv_sec = time_out,
            .tv_usec = 0,
        };
        setsockopt(socket_handl, SOL_SOCKET, SO_RCVTIMEO, &timeout_val, sizeof(timeout_val));
    }

    ssize_t read_len = recvfrom(socket_handl, dst, *len, 0, (struct sockaddr *)source_addr, &addr_len);
    if (read_len < 0)
    {
        if (errno != EAGAIN)
            ESP_LOGE(TAG, "multicast recvfrom failed: errno %d", errno);
        return false;
    }
    else if (read_len == 0)
        close_socket();
    *len = read_len;
    return true;
}

#define MDNS_MULTICAST_ADDR "224.0.0.251"
#define MDNS_PORT 5353

mDNS::mDNS() : UDP_MultiCast()
{
    TAG = (char *)calloc(1, sizeof("MDNS"));
    memcpy(TAG, "MDNS", sizeof("MDNS"));
    inet_pton(AF_INET, "255.255.255.0", &mask);
}

mDNS::~mDNS()
{
    free(TAG);
}
bool mDNS::init()
{
    if (socket_handl == INVALID_SOCKET)
        return server_create(MDNS_MULTICAST_ADDR, MDNS_PORT);
    return true;
}
bool mDNS::ask_if_same_devic_name()
{
    char query[50] = {};
    mdns_header_t *header = (mdns_header_t *)query;
    header->id = htons(1);
    header->flags = htons(0x0000);
    header->questions = htons(1);
    header->answers = htons(0);
    header->auth_rr = htons(0);
    header->add_rr = htons(0);
    char *ptr = query + sizeof(mdns_header_t);

    char *name = (char *)calloc(1, sizeof(MDNS_HOST_NAME) + 1);
    size_t len = strToNameFormat(MDNS_HOST_NAME, sizeof(MDNS_HOST_NAME), name);
    memcpy(ptr, name, len);
    free(name);
    if (len == 0)
        return false;
    ptr += len;
    *(uint16_t *)ptr = htons(0x0001);
    ptr += 2;
    *(uint16_t *)ptr = htons(0x0001);
    ptr += 2;
    send(query, ptr - query);

    return true;
}

void mDNS::listen()
{
    char buf[100] = {};
    size_t len = sizeof(buf);
    struct sockaddr_in from_addr;
    if (recv(buf, &len, &from_addr))
    {
        mdns_header_t *r_header = (mdns_header_t *)buf;
        if (ntohs(r_header->flags) & 0x8000)
            return;
        if (ntohs(r_header->questions) > 1)
            return;
        if (ntohs(r_header->answers) != 0)
            return;
        char *ptr = ((char *)r_header) + sizeof(mdns_header_t);
        char *temp_ptr = ptr;
        const char *name = MDNS_HOST_NAME;
        size_t index = 0;
        while (true)
        {
            uint8_t len = *temp_ptr;
            if (len == 0)
                break;
            temp_ptr++;
            for (uint8_t i = 0; i < len; temp_ptr++, i++, index++)
                if (*temp_ptr != name[index])
                {
                    index = 0;
                    break;
                }
            index++;
        }
        if (*temp_ptr != 0 || index == 0)
            return;

        char response[26 + sizeof(MDNS_HOST_NAME) + 5] = {};
        mdns_header_t *header = (mdns_header_t *)response;
        header->id = r_header->id;
        header->flags = htons(0x8400);
        header->questions = htons(0);
        header->answers = htons(1);
        temp_ptr = response + sizeof(mdns_header_t);
        memcpy(temp_ptr, ptr, index);
        temp_ptr += index + 1;
        *(uint16_t *)temp_ptr = htons(0x0001);
        temp_ptr += 2;
        *(uint16_t *)temp_ptr = htons(0x0001);
        temp_ptr += 2;
        *(uint32_t *)temp_ptr = htonl(60);
        temp_ptr += 4;
        *(uint16_t *)temp_ptr = htons(4);
        temp_ptr += 2;

        esp_netif_ip_info_t ap_local_addr, sta_local_addr;
        get_ip_info(&ap_local_addr, &sta_local_addr);
        // printf("AP:%ld:%ld\n", ap_local_addr.ip.addr & mask, from_addr.sin_addr.s_addr & mask);
        // printf("STA:%ld:%ld\n", sta_local_addr.ip.addr & mask, from_addr.sin_addr.s_addr & mask);
        char ip_str[16] = {};

        if ((ap_local_addr.ip.addr & mask) == (from_addr.sin_addr.s_addr & mask))
        {
            *(uint32_t *)temp_ptr = (ap_local_addr.ip.addr);
            inet_ntop(AF_INET, &ap_local_addr.ip.addr, ip_str, sizeof(ip_str));
        }
        else
        {
            *(uint32_t *)temp_ptr = (sta_local_addr.ip.addr);
            inet_ntop(AF_INET, &sta_local_addr.ip.addr, ip_str, sizeof(ip_str));
        }
        temp_ptr += 4;
        send(response, temp_ptr - response);

        ESP_LOGI(TAG, "From :%s", ip_str);
    }
}

void mDNS::get_ip_info(void *ap_addr_info, void *sta_addr_info)
{
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), (esp_netif_ip_info_t *)ap_addr_info);
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), (esp_netif_ip_info_t *)sta_addr_info);

    // char ip_str[16] = {};
    // inet_ntop(AF_INET, &info->ip, ip_str, sizeof(ip_str));
    // printf("IP:%s\n", ip_str);
}
size_t mDNS::strToNameFormat(const char *src, size_t len, char *dst)
{
    if (*src == 0 || *src == '.')
        return 0;
    char *src_ptr = (char *)src;
    char *dst_ptr = dst;
    size_t index = 1;
    for (size_t i = 0; i < len; i++, index++)
    {
        if (src[i] == '.' || src[i] == '\0')
        {
            *dst_ptr++ = (unsigned char)index - 1;
            memcpy(dst_ptr, src_ptr, index - 1);
            dst_ptr += index - 1;
            src_ptr += index;
            index = 0;
        }
    }
    *dst_ptr = 0;
    /*for (size_t i = 0; i < dst_ptr - dst; i++)
        printf("%02x ", dst[i]);
    printf("\n");*/

    return dst_ptr - dst + 1;
}
