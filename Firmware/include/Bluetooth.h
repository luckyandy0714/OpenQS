#pragma once
#include <stdio.h>

#define BLUETOOTH_RECEIVER_BUFFER_SIZE 512

class Bluetooth
{
public:
    typedef void (*Connected_Callback)(bool Connected);
    static void setConnected_Callback(Connected_Callback callback);
    static Connected_Callback getConnected_Callback();

    Bluetooth();
    ~Bluetooth();
    void init();
    size_t getAvailableBufferSize();
    size_t readBytes(void *dst, size_t len);
    void sendBytes(const void *src, size_t len);

private:
    static Connected_Callback bt_connect_event_Callback;
};
