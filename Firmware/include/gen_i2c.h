#pragma once

#include "stdint.h"
#include "stddef.h"

#include "driver/i2c_types.h"

class gen_i2c
{
private:
    i2c_port_t i2c_port;
    uint16_t device_addr;

public:
    gen_i2c() {};
    gen_i2c(int SDA_PIN, int SCL_PIN, uint32_t FREQ_HZ, uint16_t device_addr, i2c_port_t i2c_port);
    ~gen_i2c();

    bool i2c_probe();

    void write_bytes(uint8_t write_addr, size_t write_length, const uint8_t *write_data);
    void read_bytes(uint8_t read_addr, size_t read_length, uint8_t *read_data);
    void write_byte(uint8_t write_addr, const uint8_t write_data);
    void read_byte(uint8_t read_addr, uint8_t *read_data);
};