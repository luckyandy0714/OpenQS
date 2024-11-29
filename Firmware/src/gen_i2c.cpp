#include "gen_i2c.h"
#include <string.h>
#ifdef ESP_PLATFORM
#define ENVIRONMENT "ESP-IDF"
#include "driver/i2c.h"
// #include "driver/i2c_master.h"

gen_i2c::gen_i2c(int SDA_PIN, int SCL_PIN, uint32_t FREQ_HZ, uint16_t device_addr, i2c_port_t i2c_port = I2C_NUM_0)
{
    this->device_addr = device_addr;
    this->i2c_port = i2c_port;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master = FREQ_HZ,
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0));
    /*
    i2c_master_bus_config_t i2c_conf{
        .i2c_port = 0,
        .sda_io_num = (gpio_num_t)SDA_PIN,
        .scl_io_num = (gpio_num_t)SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 10,
        .flags = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_conf, &bus_handle));

    i2c_device_config_t dev_config{
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));*/
}
gen_i2c::~gen_i2c()
{
    /*if (dev_handle)
        i2c_master_bus_rm_device(dev_handle);
    if (bus_handle)
        i2c_del_master_bus(bus_handle);*/
}
bool gen_i2c::i2c_probe()
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd_handle);
    bool result = i2c_master_cmd_begin(i2c_port, cmd_handle, 100 / portTICK_PERIOD_MS) == ESP_OK;
    i2c_cmd_link_delete(cmd_handle);
    return result;
    // auto err = i2c_master_probe(bus_handle, dev_add, 100);
    // ESP_ERROR_CHECK(err);
    // return err == ESP_OK;
}
void gen_i2c::write_bytes(uint8_t write_addr, size_t write_length, const uint8_t *write_data)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, write_addr, true);
    for (size_t i = 0; i < write_length; i++)
        i2c_master_write_byte(cmd_handle, write_data[i], true);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(i2c_port, cmd_handle, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    // WriteBuffer[0] = write_addr;
    // memcpy(&WriteBuffer[1], write_data, write_length);
    // i2c_master_transmit(dev_handle, WriteBuffer, write_length + 1, -1);
}
void gen_i2c::read_bytes(uint8_t read_addr, size_t read_length, uint8_t *read_data)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, read_addr, true);
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (device_addr << 1) | I2C_MASTER_READ, true);
    for (size_t i = 0; i < read_length; i++)
        i2c_master_read_byte(cmd_handle, read_data + i, (i + 1 >= read_length) ? I2C_MASTER_NACK : I2C_MASTER_ACK);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(i2c_port, cmd_handle, (100 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd_handle);
    // i2c_master_transmit_receive(dev_handle, &read_addr, 1, read_data, read_length, -1);
}
void gen_i2c::write_byte(uint8_t write_addr, const uint8_t write_data)
{
    write_bytes(write_addr, 1, &write_data);
}

void gen_i2c::read_byte(uint8_t read_addr, uint8_t *read_data)
{
    read_bytes(read_addr, 1, read_data);
}

#elif defined(ARDUINO)
#define ENVIRONMENT "ARDUINO"
#else
#error "Unknown Environment"
#endif
