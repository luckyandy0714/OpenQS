#pragma once
#include <stdio.h>

class Wifi
{
private:
public:
    Wifi();
    ~Wifi();
    void init();
    void scan(bool sync = false);
    void connect(const char *ssid, const char *password, bool sync = false);
    void disconnect();
    static bool wait_connect(uint32_t time_out = 0);
    static bool wait_AP_connect(uint32_t time_out = 0);
};
