#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <driver/dac.h>
#include <driver/adc.h>

#include <esp_adc_cal.h>
#include <esp_timer.h>

#include "OTA.h"
#include "bluetooth.h"
#include "WIFI.h"
#include "CustomSocket.h"
#include "storage.h"
#include "hx711.h"
#include "mpu6050.h"

#include "VARIABLE.h"
#include "Config.h"
#include "Math.h"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define FREQ_TO_PERIOD(FREQ) (1000.0 / (FREQ))
#define PERIOD_TO_FREQ(PERIOD) (1000.0 / (PERIOD))
#define FREQ_TO_RPM(FREQ) ((FREQ) * 120000.0)
#define RPM_TO_FREQ(RPM) ((RPM) / 120000.0)
#define CALCULATE_DELTA(DELTA, DELTA_T) ((DELTA) / (DELTA_T))

#define SLEEP(TIME_MS) vTaskDelay(pdMS_TO_TICKS(TIME_MS))
#define SLEEPTICKS(NUM) vTaskDelay(NUM)
#define TIME (esp_timer_get_time() / 1000)
#define MICROS_TIME (esp_timer_get_time())

Variable variable = {
    .CurrentState = {
        .EngineSpeed = 0.0f,
        .EngineSpeed_Delta = 0.0f,
        .GearState = 0,
        .a = 0,
        .Quate = {
            .W = 0.0f,
            .X = 0.0f,
            .Y = 0.0f,
            .Z = 0.0f,
        },
        .Sensor = 0.0f,
        .Temperature = 0.0f,
        .Voltage = 0.0f,
    },
    .BitConfig = {
        .CUTOFF_Positive_Enable = true,
        .CUTOFF_Negative_Enable = false,
        .CUTOFF_LimitOfMinGear = false,
        .CUTOFF_LimitOfMaxGear = false,
        .CUTOFF_Smooth_Curve = false,
        .EngineSpeed_Ratio = true,
        .EngineSpeed_Delta_Ratio = true,
        .GearState_Ratio = true,
        .EngineSpeed_Half = false,
        .Sensor_Direction_Reverse = false,
        .Silent_Mode = false,
        .EngineStart_Delay = false,
        .IMU_Enable = false,
        .b = false,
        .c = false,
        .d = false,
    },
    .ValueConfig = {
        .CUTOFF_Time = 80,
        .CUTOFF_BeforeDelayTime = 20,
        .CUTOFF_AfterDelayTime = 100,
        .e = 0,
        .CUTOFF_SmoothTime_T1 = 0.0f,
        .CUTOFF_SmoothTime_T2 = 1.0f,
        .Sensor_Positive_Threshold = 100.0f,
        .Sensor_Negative_Threshold = -100.0f,
        .Sensor_Offset = 0.0f,
        .Sensor_Scale = 1.0f,

        .EngineSpeed_Ratio_Map_Label = {
            {1800, 2000, 3500, 4000, 5000, 6000, 8000, 10000, 12000, 14000},
            {1800, 2000, 3500, 4000, 5000, 6000, 8000, 10000, 12000, 14000},
        },
        .EngineSpeed_Ratio_Map = {
            {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
            {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
        },
        .EngineSpeed_Delta_Ratio_Map_Label = {
            {-10000, -6000, -2000, -1000, 0, 1000, 2000, 4000, 6000, 10000},
            {-10000, -6000, -2000, -1000, 0, 1000, 2000, 4000, 6000, 10000},
        },
        .EngineSpeed_Delta_Ratio_Map = {
            {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
            {100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
        },
        .GearState_Ratio_Map = {
            {100, 100, 100, 100, 100},
            {100, 100, 100, 100, 100},
        },
    },
};
const char *variable_ptr = (char *)&variable;
const char *variable_config_ptr = variable_ptr + sizeof(Variable::CURRENTSTATE);
size_t variable_config_size = sizeof(Variable) - sizeof(Variable::CURRENTSTATE);

Wifi wifi;
Bluetooth bt;

void mdns_task(void *pvParameter)
{
    SLEEP(100);
    mDNS mdns_server;
    while (true)
    {
        if (mdns_server.init())
            mdns_server.listen();
        SLEEP(1000);
    }
}

void ota_task(void *pvParameter)
{
    SLEEP(100);
    OTA_init();
    OTA_diagnostic(OTA_DIAGNOSTIC_PIN);

    SLEEP(100);
    Socket socket_a(UDP, SERVER);
    Socket socket_b(TCP, CLIENT);

    while (true)
    {
        SLEEP(100);
        if (!socket_a.server_create(OTA_UDP_RECEIVER_PORT))
            continue;
        struct
        {
            uint32_t command;
            uint32_t host_port;
            size_t content_size;
            char hash_code[OTA_VERIFY_HASH_DECODE_LENGTH];
        } packet;
        uint32_t host_ip = 0;
        bool error = true;
        while (true)
        {
            SLEEP(100);
            char packet_buffer[OTA_UDP_BUFFER_SIZE] = {};
            size_t len = sizeof(packet_buffer);
            if (socket_a.recv(packet_buffer, &len))
            {
                packet_buffer[len] = '\0';
                size_t count = 0;
                for (size_t i = 0; i < sizeof(packet_buffer) && packet_buffer[i] != '\0'; i++)
                    if (packet_buffer[i] == ' ')
                        count++;
                if (count != 3)
                    break;
                if (sscanf(packet_buffer, "%ld %ld %d %s", &packet.command, &packet.host_port, &packet.content_size, packet.hash_code) != 4)
                    break;
                // printf("command:%ld,host_port:%ld,content_size:%d,hash_code:%s\n", packet.command, packet.host_port, packet.content_size, packet.hash_code);

                host_ip = socket_a.get_server_addr();
                socket_a.send("OK", sizeof("OK") - 1);
                socket_a.close_socket();
                error = false;
                break;
            }
        }
        if (error)
            continue;
        socket_b.client_connect(host_ip, packet.host_port, true);
        char *b_data = (char *)calloc(1, OTA_TCP_BUFFER_SIZE);
        size_t b_len = OTA_TCP_BUFFER_SIZE;
        size_t all_len = 0;
        error = false;
        OTA_start_str(packet.content_size, packet.hash_code);
        while (true)
        {
            SLEEP(1);
            if (!socket_b.recv(b_data, &b_len))
                break;
            all_len += b_len;
            socket_b.send("OK", sizeof("OK") - 1);
            if (!OTA_updata(b_data, b_len))
            {
                error = true;
                break;
            }
        }
        OTA_finish();
        free(b_data);
        socket_b.close_socket();
        if (error)
            continue;

        if (!OTA_verify(true))
            continue;
    }
}

typedef struct
{
    enum OPERATE
    {
        Read,
        Write,
        Delete,
    } Operate;
} StoragePackets_t;
typedef struct
{
    uint32_t StatePeriod;
    uint32_t BlinkPeriod;
    uint32_t BlinkTime;
    uint8_t BlinkNumber;

} BlinkPackets_t;
typedef struct
{
    uint16_t Frequency;
    uint16_t Ring_Period;
    uint8_t Ring_Encoded : BUZZER_ENCODE_RESOLUTION;
} BuzzerPackets_t;
QueueHandle_t StorageQueue, BlinkQueue, BuzzerQueue;
void queue_task(void *pvParameter)
{
    StorageQueue = xQueueCreate(3, sizeof(StoragePackets_t));

    BlinkQueue = xQueueCreate(2, sizeof(BlinkPackets_t));
    BuzzerQueue = xQueueCreate(4, sizeof(BuzzerPackets_t));

    Storage config(STORAGE_CONFIG_FILNAME);

    gpio_config_t blink_pin_conf = {
        .pin_bit_mask = (1ULL << BLINK_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&blink_pin_conf);

    ledc_timer_config_t buzzer_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = BUZZER_DEFAULT_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ledc_channel_config_t buzzer_timer_channel = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = 0,
    };
    ledc_timer_config(&buzzer_timer);
    ledc_channel_config(&buzzer_timer_channel);

    BlinkPackets_t blink_packets = {
        .StatePeriod = 2000,
        .BlinkPeriod = 200,
        .BlinkTime = 10,
        .BlinkNumber = 0,
    };
    BuzzerPackets_t buzzer_packets = {
        .Frequency = BUZZER_DEFAULT_FREQUENCY,
        .Ring_Period = 50,
        .Ring_Encoded = 0b00000000,
    };

    int64_t StatePeriod_timer = 0, BlinkTime_timer = 0, BlinkPeriod_timer = 0, Ring_Period_timer = 0;
    uint32_t BlinkNumber_counter = 0, Ring_Encoded_index = 0;
    bool Blink_flag = false, duty_flag = false, buzzer_flag = false;

    while (true)
    {
        StoragePackets_t temp_storage_packsts;
        if (xQueueReceive(StorageQueue, &temp_storage_packsts, 0))
        {
            char *temp = (char *)calloc(variable_config_size, 1);
            switch (temp_storage_packsts.Operate)
            {
            case StoragePackets_t::Read:
                if (config.read(temp, variable_config_size))
                    memcpy((void *)variable_config_ptr, temp, variable_config_size);
                break;
            case StoragePackets_t::Write:
                memcpy(temp, (void *)variable_config_ptr, variable_config_size);
                config.write(temp, variable_config_size);
                break;
            case StoragePackets_t::Delete:
                config.deleteFile();
                // config.format();
                break;
            default:
                break;
            }
            free(temp);
        }
        BlinkPackets_t temp_blink_packets;
        if (xQueueReceive(BlinkQueue, &temp_blink_packets, 0))
        {
            if (temp_blink_packets.BlinkPeriod * temp_blink_packets.BlinkNumber > temp_blink_packets.StatePeriod)
                temp_blink_packets.BlinkPeriod = temp_blink_packets.StatePeriod / temp_blink_packets.BlinkNumber;
            temp_blink_packets.BlinkTime = MIN(temp_blink_packets.BlinkTime, temp_blink_packets.BlinkPeriod * 0.9);
            for (size_t i = 0; i < sizeof(BlinkPackets_t); i++)
                if (((char *)&temp_blink_packets)[i] != 0)
                    ((char *)&blink_packets)[i] = ((char *)&temp_blink_packets)[i];
        }
        int64_t current_time = TIME;

        if (current_time > StatePeriod_timer)
        {
            StatePeriod_timer = current_time + blink_packets.StatePeriod;
            BlinkNumber_counter = blink_packets.BlinkNumber;
        }
        if (BlinkNumber_counter != 0)
        {
            if (!Blink_flag && current_time > BlinkPeriod_timer)
            {
                BlinkTime_timer = current_time + blink_packets.BlinkTime;
                BlinkPeriod_timer = current_time + blink_packets.BlinkPeriod;
                gpio_set_level((gpio_num_t)BLINK_PIN, 1);
                Blink_flag = true;
            }
            else if (Blink_flag && current_time > BlinkTime_timer)
            {
                gpio_set_level((gpio_num_t)BLINK_PIN, 0);
                Blink_flag = false;
                BlinkNumber_counter--;
            }
        }
        if (!buzzer_flag)
        {
            BuzzerPackets_t temp_buzzer_packets;
            if (xQueueReceive(BuzzerQueue, &temp_buzzer_packets, 0))
            {
                if (!variable.BitConfig.Silent_Mode)
                {
                    for (size_t i = 0; i < sizeof(BuzzerPackets_t); i++)
                        ((char *)&buzzer_packets)[i] = ((char *)&temp_buzzer_packets)[i];

                    ledc_set_freq(buzzer_timer_channel.speed_mode, buzzer_timer_channel.timer_sel,
                                  (buzzer_packets.Frequency != 0) ? buzzer_packets.Frequency : BUZZER_DEFAULT_FREQUENCY);

                    buzzer_packets.Ring_Period *= 0.9;
                    Ring_Encoded_index = 0;
                    buzzer_flag = true;
                }
            }
        }
        else if (Ring_Encoded_index >= BUZZER_ENCODE_RESOLUTION)
            buzzer_flag = false;
        else if (!duty_flag)
        {
            Ring_Period_timer = current_time + buzzer_packets.Ring_Period;
            if (buzzer_packets.Ring_Encoded & (1 << Ring_Encoded_index))
            {
                ledc_set_duty(buzzer_timer_channel.speed_mode, buzzer_timer_channel.channel, 1);
                ledc_update_duty(buzzer_timer_channel.speed_mode, buzzer_timer_channel.channel);
            }
            duty_flag = true;
        }
        else if (duty_flag && current_time > Ring_Period_timer)
        {
            Ring_Encoded_index++;
            if (!(buzzer_packets.Ring_Encoded & (1 << Ring_Encoded_index)) || Ring_Encoded_index >= BUZZER_ENCODE_RESOLUTION)
            {
                ledc_set_duty(buzzer_timer_channel.speed_mode, buzzer_timer_channel.channel, 0);
                ledc_update_duty(buzzer_timer_channel.speed_mode, buzzer_timer_channel.channel);
            }
            duty_flag = false;
        }

        SLEEP(1);
    }
}
void uart_task(void *pvParams)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0));
    Storage config(STORAGE_CONFIG_FILNAME);
    char data[255];

    while (true)
    {
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, (size_t *)&length));
        if (length > 0)
        {
            length = uart_read_bytes(UART_NUM_0, data, length, 1000);
            data[length] = '\0';
            printf("Received data: %s", data);
            if (!strcmp(data, "led_on\r\n"))
                gpio_set_level((gpio_num_t)BLINK_PIN, 1);
            else if (!strcmp(data, "led_off\r\n"))
                gpio_set_level((gpio_num_t)BLINK_PIN, 0);
            else if (!strcmp(data, "s_path\r\n"))
            {
                printf("storage file path: %s\n", config.getPath());
            }
            else if (!strcmp(data, "s_version\r\n"))
            {
                printf("storage file version: %s \n", config.getPath());
            }
            else if (!strcmp(data, "s_size\r\n"))
            {
                printf("storage file size: %d bytes\n", config.fileSize());
            }
            else if (!strcmp(data, "s_preview\r\n"))
            {
                size_t size = config.fileSize();
                char *temp = (char *)calloc(size, 1);
                if (config.read(temp, size))
                {
                    printf("read storage file size: %d bytes\nread data: \n", size);
                    for (size_t i = 0; i < size; i++)
                        printf("%d,", temp[i]);
                    printf("\n");
                    fflush(stdout);
                }
                else
                    printf("read storage file failed\n");
                free(temp);
            }
            else if (!strcmp(data, "s_import\r\n"))
            {
                char *temp = (char *)calloc(variable_config_size, 1);
                if (config.read(temp, variable_config_size))
                {
                    printf("storage file size: %d bytes\nread data: \n", variable_config_size);
                    memcpy((void *)variable_config_ptr, temp, variable_config_size);
                    for (size_t i = 0; i < variable_config_size; i++)
                        printf("%d,", temp[i]);
                    printf("\n");
                    fflush(stdout);
                }
                else
                    printf("read storage file failed\n");
                free(temp);
            }
            else if (!strcmp(data, "s_export\r\n"))
            {
                char *temp = (char *)calloc(variable_config_size, 1);
                memcpy(temp, (void *)variable_config_ptr, variable_config_size);
                if (config.write(temp, variable_config_size))
                {
                    printf("write storage file size: %d bytes\nwrite data: \n", variable_config_size);
                    for (size_t i = 0; i < variable_config_size; i++)
                        printf("%d,", temp[i]);
                    printf("\n");
                    fflush(stdout);
                }
                else
                    printf("write storage file failed\n");
                free(temp);
            }
            else if (!strcmp(data, "s_format\r\n"))
            {
                config.format();
            }
            else if (!strcmp(data, "test\r\n"))
            {
                char *temp = (char *)calloc(sizeof(Variable), 1);
                memcpy(temp, (void *)&variable, sizeof(Variable));
                for (size_t i = 0; i < sizeof(Variable); i++)
                    printf("%d,", temp[i]);
                printf("\n");
                fflush(stdout);
                free(temp);
            }
            else if (!strcmp(data, "help\r\n"))
            {
                const char *help_str =
                    "\"led_on\": Turn on the LED.\n"
                    "\"led_off\": Turn off the LED.\n"
                    "\"h_version\": Display the hardware version.\n"
                    "\"s_path\": Show the path of the storage file.\n"
                    "\"s_version\": Display the storage version.\n"
                    "\"s_size\": Show the size of the storage file.\n"
                    "\"s_preview\": Preview the data of the storage file.\n"
                    "\"s_import\": Import config from the storage file.\n"
                    "\"s_export\": Export config to the storage file.\n"
                    "\"s_format\": Format the storage flash memory.\n";
                printf("%s\n", help_str);
            }
        }
        SLEEP(100);
    };
}
Hx711 *sensor_ptr = NULL;
void sensor_task(void *pvParameter)
{
    Hx711 sensor(HX711_CLOCK_PIN, HX711_DOUT_PIN, GAIN_128);
    sensor.start();
    sensor.readbyISR();
    sensor.setAverage(HX711_AVERAGE_NUM);
    sensor_ptr = &sensor;
    ParseCommand::setResetSensorOffsetCommandCallback([]
                                                      { 
        float temp = sensor_ptr->GetConvertData() * (variable.BitConfig.Sensor_Direction_Reverse ? -1 : 1)*-1;
        variable.ValueConfig.Sensor_Offset=temp;
        sensor_ptr->setOffset(temp); });
    SLEEP(100);
    while (true)
    {
        // sensor.read();
        sensor.setOffset(variable.ValueConfig.Sensor_Offset);
        sensor.setScale(variable.ValueConfig.Sensor_Scale);
        float temp = sensor.GetConvertData() * (variable.BitConfig.Sensor_Direction_Reverse ? -1 : 1);
        float range = temp - (ABS(temp) * 0.1);
        variable.CurrentState.Sensor = CONSTRAIN(temp, temp - range, temp + range);
        // variable.CurrentState.Sensor = sensor.GetRawData();
        //  printf("Sensor: %f\n", variable.CurrentState.Sensor);
        SLEEP(10);
    }
}
void adc_task(void *pvParameter)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TEMP_PIN, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(VOLT_PIN, ADC_ATTEN_DB_12);
    esp_adc_cal_characteristics_t *adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 0, adc_chars);
    while (true)
    {
        variable.CurrentState.Temperature = ADCTOTEMP(esp_adc_cal_raw_to_voltage(adc1_get_raw(TEMP_PIN), adc_chars));
        variable.CurrentState.Voltage = ADCTOVOLT(esp_adc_cal_raw_to_voltage(adc1_get_raw(VOLT_PIN), adc_chars));
        SLEEP(FREQ_TO_PERIOD(ADC_SAMPLERATE));
    }
}

void imu_task(void *pvParameter)
{
    mpu6050 mpu;
    if (mpu.initMPU6050(MPU_SDA_PIN, MPU_SCL_PIN, I2C_CLACK_FREQ) != SUCCESS)
    {
        printf("Failed to initMPU6050\n");
        vTaskDelete(NULL);
    }
    printf("Success to initMPU6050\n");

    FIFOPacket packet;
    SLEEP(100);
    printf("\r                                                                                                                                          \r");
    while (1)
    {
        /*Vec3i acc, gyro;
        mpu.getRawAcc(&acc);
        mpu.getRawGyro(&gyro);
        printf("Acc: %6d, %6d, %6d, ", acc.Vec_0, acc.Vec_1, acc.Vec_2);
        printf("Gyro: %6d, %6d, %6d\r\n", gyro.Vec_0, gyro.Vec_1, gyro.Vec_2);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        continue;*/

        int16_t count = mpu.getFIFOCount();
        if (count >= 1)
            mpu.dmpReadFIFOPacket(&packet);
        // printf("Quate: %+6.3f, %+6.3f, %+6.3f, %+6.3f; ", packet.Quate_W, packet.Quate_X, packet.Quate_Y, packet.Quate_Z);
        // printf("Acc: %+6.3f, %+6.3f, %+6.3f; ", packet.Accel_X, packet.Accel_Y, packet.Accel_Z);
        // printf("Gyro: %+8.3f, %+8.3f, %+8.3f\r", packet.Gyro_X, packet.Gyro_Y, packet.Gyro_Z);
        // fflush(stdout);
        if (count > 10)
            mpu.resetFIFO();
        SLEEP(10);
    }
}
void cutOff(uint16_t cutoff_time)
{
    cutoff_time = MIN(MAX(cutoff_time, 10), 1000);
    int64_t timer = 0;
    auto t1 = cutoff_time * ((!variable.BitConfig.CUTOFF_Smooth_Curve) ? 0.0 : variable.ValueConfig.CUTOFF_SmoothTime_T1);
    auto t2 = cutoff_time * ((!variable.BitConfig.CUTOFF_Smooth_Curve) ? 0.0 : (1.0 - variable.ValueConfig.CUTOFF_SmoothTime_T2));
    auto tp = MAX(cutoff_time - t1 - t2, 0);
    // Begin CUTOFF to T1 rising Curve
    auto cuttimer = timer = TIME;
    while (true)
    {
        cuttimer = TIME - timer;
        if (cuttimer >= t1)
            break;
        dac_output_voltage(CUTOFF_PIN, INTERPOLATION(MIN(cuttimer, t1), 0, t1, 0, 255));
    }
    dac_output_voltage(CUTOFF_PIN, 255);
    // T1 to T2 (TPeak)
    cuttimer = TIME + tp;
    while (TIME < cuttimer)
        SLEEPTICKS(1);
    // T2 to end CUTOFF falling Curve
    timer = TIME;
    while (true)
    {
        cuttimer = TIME - timer;
        if (cuttimer >= t2)
            break;
        dac_output_voltage(CUTOFF_PIN, INTERPOLATION(MIN(cuttimer, t2), 0, t2, 255, 0));
    }
    dac_output_voltage(CUTOFF_PIN, 0);
}
static_assert(sizeof(Variable) + 4 < BLUETOOTH_COMMAND_BUFFER_SIZE, "BLUETOOTH_BUFFER_SIZE must be larger than VARIABLE size");

void bluetooth_acceptor_task(void *pvParameter)
{
    bt.init();
    bt.setConnected_Callback([](bool Connected)
                             {
         BlinkPackets_t blink_packets = {
        .StatePeriod = 10000,
        .BlinkPeriod = 200,
        .BlinkTime = 10,
        .BlinkNumber = 2,
    };
    BuzzerPackets_t buzzer_packets = {
        .Frequency = BUZZER_DEFAULT_FREQUENCY,
        .Ring_Period = 40,
        .Ring_Encoded = 0b00000001,
    };
    if (Connected)
    {
        blink_packets.BlinkNumber = 0;
        buzzer_packets.Ring_Encoded = 0b00010001;
    }
    ParseCommand::clear();
    xQueueSend(BlinkQueue, (void *)&blink_packets, 0);
    xQueueSend(BuzzerQueue, (void *)&buzzer_packets, portTICK_PERIOD_MS); });

    ParseCommand::setVariable_ptr(variable_ptr);
    ParseCommand::setTransmitCallback([](const void *src, const size_t len)
                                      { bt.sendBytes(src, len); });
    ParseCommand::setWriteAddressCommandCallback([]
                                                 {
    StoragePackets_t storage_packets = {.Operate = StoragePackets_t::Write};
    xQueueSend(StorageQueue, (void *)&storage_packets, 0); });

    ParseCommand::setEnableQSCommandCallback([](bool enable) {});
    ParseCommand::setResetCommandCallback([]
                                          { esp_restart(); });
    ParseCommand::setResetDefaultVariableCommandCallback([]
                                                         {
    StoragePackets_t storage_packets = {.Operate = StoragePackets_t::Delete};
    xQueueSend(StorageQueue, (void *)&storage_packets, 0); });
    ParseCommand::setManualTriggerCommandCallbackCallback([](uint16_t cutoff_time)
                                                          {
    cutOff(cutoff_time); 
    CUTOFF_LOG log = {
        .EngineSpeed = variable.CurrentState.EngineSpeed,
        .EngineSpeed_Delta = variable.CurrentState.EngineSpeed_Delta,
        .GearState = variable.CurrentState.GearState,
        .CutOff_Time = (uint16_t)cutoff_time,
        };
    ParseCommand::transmit_cutoff_log_Command(log); });
    // ParseCommand::setSetWifiCommandCallbackCallback([](const char * id,const char *password) {});

    while (true)
    {
        uint16_t availableBufferSize = bt.getAvailableBufferSize();
        char *buffer = NULL;
        size_t actually_size = 0;

        if (availableBufferSize > 0)
        {
            buffer = (char *)malloc(availableBufferSize);
            actually_size = bt.readBytes(buffer, availableBufferSize);
        }
        if (!ParseCommand::parseCommand(buffer, actually_size))
            SLEEP(100);

        if (buffer)
            free(buffer);
    }
}

volatile int64_t ensp_isr_timer = 0;
volatile int64_t ensp_isr_pulse_timer = 0;
volatile int64_t ensp_isr_sampling_timer = 0;
volatile int64_t ensp_isr_max_speed_timer = 0;
volatile bool ensp_isr_receive_pulse_flag = false;

void IRAM_ATTR enginespeed_isr_handler(void *arg)
{
    ensp_isr_timer = MICROS_TIME;
    if (ensp_isr_timer >= ensp_isr_sampling_timer)
    {
        ensp_isr_sampling_timer = ensp_isr_timer + (FREQ_TO_PERIOD(ENGINESPEED_MAXIMUM_SAMPLERATE) * 1000);
        if (!ensp_isr_receive_pulse_flag)
        {
            ensp_isr_receive_pulse_flag = true;
            ensp_isr_pulse_timer = ensp_isr_timer;
            ensp_isr_max_speed_timer = ensp_isr_timer + FREQ_TO_PERIOD(RPM_TO_FREQ(MAXIMUM_ENGINESPEED));
            return;
        }
    }
    if (ensp_isr_receive_pulse_flag)
    {
        if (ensp_isr_timer < ensp_isr_max_speed_timer)
            return;
        ensp_isr_receive_pulse_flag = false;
        Variable *Var = static_cast<Variable *>(arg);
        ensp_isr_pulse_timer = ensp_isr_timer - ensp_isr_pulse_timer;
        if (ensp_isr_pulse_timer > FREQ_TO_PERIOD(RPM_TO_FREQ(MINIMUM_ENGINESPEED)))
            Var->CurrentState.EngineSpeed = 0;
        else
            Var->CurrentState.EngineSpeed =
                FREQ_TO_RPM(PERIOD_TO_FREQ(ensp_isr_pulse_timer)) * (Var->BitConfig.EngineSpeed_Half ? 0.5 : 1.0);
    }
}
void delta_task(void *pvParameter)
{
    int64_t last_delta_timer = MICROS_TIME;
    float last_engine_speed = 0;

    while (true)
    {
        if (variable.CurrentState.EngineSpeed == 0)
        {
            last_engine_speed = 0;
            variable.CurrentState.EngineSpeed_Delta = 0;
        }
        else
        {
            variable.CurrentState.EngineSpeed_Delta =
                CALCULATE_DELTA(variable.CurrentState.EngineSpeed - last_engine_speed,
                                (MICROS_TIME - last_delta_timer) / 1000000.0);
            last_engine_speed = variable.CurrentState.EngineSpeed;
            last_delta_timer = MICROS_TIME;
        }

        SLEEP(100);
    }
}
void main_task(void *pvParameter)
{
    SLEEP(1000);
    gpio_config_t engineSpeedPin_conf = {
        .pin_bit_mask = (1ULL << ENGINESPEED_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&engineSpeedPin_conf);
    gpio_isr_handler_add((gpio_num_t)ENGINESPEED_PIN, enginespeed_isr_handler, (void *)&variable);

    bool sensor_lock_flag = true;
    bool sensor_threshold_flag = true;
    bool sensor_dir_negative = false;
    int64_t timer = 0;
    dac_output_enable(CUTOFF_PIN);
    while (true)
    {
        if (!sensor_threshold_flag)
        {
            // Sensor Positive Threshold
            if (variable.CurrentState.Sensor > variable.ValueConfig.Sensor_Positive_Threshold)
            {
                sensor_dir_negative = !(sensor_threshold_flag = true);
            }
            // Sensor Negative Threshold
            if (variable.CurrentState.Sensor < variable.ValueConfig.Sensor_Negative_Threshold)
            {
                sensor_dir_negative = sensor_threshold_flag = true;
            }
            timer = TIME + variable.ValueConfig.CUTOFF_BeforeDelayTime;
        }
        // Sensor below the Threshold to exit CUTOFF
        else if ((!sensor_dir_negative && (variable.CurrentState.Sensor < variable.ValueConfig.Sensor_Positive_Threshold * 0.8)) ||
                 (sensor_dir_negative && (variable.CurrentState.Sensor > -variable.ValueConfig.Sensor_Positive_Threshold * 0.8)))
        {
            sensor_threshold_flag = sensor_dir_negative = sensor_lock_flag = false;
        }
        // Check if CUTOFF is Locked to prevent multiple triggers
        else if (sensor_threshold_flag && !sensor_lock_flag)
        { // Use a Timer to wait for the Sensor to stable
            if (TIME >= timer)
            {
                sensor_lock_flag = true; // Locked to prevent multiple triggers

                auto EngineSpeed = variable.CurrentState.EngineSpeed;
                auto EngineSpeedDifferential = variable.CurrentState.EngineSpeed_Delta;
                auto GearState = variable.CurrentState.GearState;

                if (GearState == 0)
                    variable.CurrentState.GearState = (sensor_dir_negative) ? 1 : 2;
                else
                    variable.CurrentState.GearState = CONSTRAIN(GearState + (sensor_dir_negative ? -1 : 1), 0, 6);

                // Verify EngineSpeed in Map range
                if (EngineSpeed > variable.ValueConfig.EngineSpeed_Ratio_Map_Label[sensor_dir_negative][0])
                { // Verify EngineSpeed in Map range
                    if (EngineSpeed <= variable.ValueConfig.EngineSpeed_Ratio_Map_Label[sensor_dir_negative][ENGINESPEED_RATIO_MAP_SIZE - 1])
                    { // Verify Config the Enable CUTOFF
                        if ((variable.BitConfig.CUTOFF_Positive_Enable && !sensor_dir_negative) || (variable.BitConfig.CUTOFF_Negative_Enable && sensor_dir_negative))
                        { // Verify GearState is Out Of the Gear range
                            if ((!(!sensor_dir_negative && GearState == (GEARSTATE_NUMBER - 1)) || !variable.BitConfig.CUTOFF_LimitOfMaxGear) &&
                                (!(sensor_dir_negative && GearState == 1) || !variable.BitConfig.CUTOFF_LimitOfMinGear))
                            {
                                float EngineSpeed_coeff = 1.0;
                                float EngineSpeedDelta_coeff = 1.0;
                                float GearState_coeff = 1.0;
                                if (variable.BitConfig.EngineSpeed_Ratio)
                                    for (uint8_t i = 0; i < ENGINESPEED_RATIO_MAP_SIZE - 1; i++)
                                    {
                                        if (EngineSpeed > variable.ValueConfig.EngineSpeed_Ratio_Map_Label[sensor_dir_negative][i] &&
                                            EngineSpeed <= variable.ValueConfig.EngineSpeed_Ratio_Map_Label[sensor_dir_negative][i + 1])
                                        {
                                            EngineSpeed_coeff = INTERPOLATION(EngineSpeed,
                                                                              (float)variable.ValueConfig.EngineSpeed_Ratio_Map_Label[sensor_dir_negative][i],
                                                                              (float)variable.ValueConfig.EngineSpeed_Ratio_Map_Label[sensor_dir_negative][i + 1],
                                                                              (float)variable.ValueConfig.EngineSpeed_Ratio_Map[sensor_dir_negative][i],
                                                                              (float)variable.ValueConfig.EngineSpeed_Ratio_Map[sensor_dir_negative][i + 1]) /
                                                                100.0;

                                            break;
                                        }
                                    }
                                if (variable.BitConfig.EngineSpeed_Delta_Ratio)
                                    for (uint8_t i = 0; i < ENGINESPEED_DELTA_RATIO_MAP_SIZE - 1; i++)
                                    {
                                        if (EngineSpeedDifferential > variable.ValueConfig.EngineSpeed_Delta_Ratio_Map_Label[sensor_dir_negative][i] &&
                                            EngineSpeedDifferential <= variable.ValueConfig.EngineSpeed_Delta_Ratio_Map_Label[sensor_dir_negative][i + 1])
                                        {
                                            EngineSpeedDelta_coeff = INTERPOLATION(EngineSpeedDifferential,
                                                                                   (float)variable.ValueConfig.EngineSpeed_Delta_Ratio_Map_Label[sensor_dir_negative][i],
                                                                                   (float)variable.ValueConfig.EngineSpeed_Delta_Ratio_Map_Label[sensor_dir_negative][i + 1],
                                                                                   (float)variable.ValueConfig.EngineSpeed_Delta_Ratio_Map[sensor_dir_negative][i],
                                                                                   (float)variable.ValueConfig.EngineSpeed_Delta_Ratio_Map[sensor_dir_negative][i + 1]) /
                                                                     100.0;
                                            break;
                                        }
                                    }
                                if (variable.BitConfig.GearState_Ratio)
                                    GearState_coeff = variable.ValueConfig.GearState_Ratio_Map[sensor_dir_negative][CONSTRAIN(((int8_t)GearState) - 1, 0, GEARSTATE_NUMBER - 1)] / 100.0;
                                // printf("a:%f,b:%f,c:%f,\n", EngineSpeed_coeff, EngineSpeedDelta_coeff, GearState_coeff);
                                uint16_t cutoff_time = (uint16_t)(((float)variable.ValueConfig.CUTOFF_Time) * EngineSpeed_coeff * EngineSpeedDelta_coeff * GearState_coeff);

                                cutOff(cutoff_time);
                                CUTOFF_LOG log = {
                                    .EngineSpeed = EngineSpeed,
                                    .EngineSpeed_Delta = EngineSpeedDifferential,
                                    .GearState = GearState,
                                    .CutOff_Time = (uint16_t)cutoff_time,
                                };
                                ParseCommand::transmit_cutoff_log_Command(log);
                            }
                        }
                    }
                } // Neutral Gear Logic
                else if (GearState == 1 && !sensor_dir_negative)
                {
                    variable.CurrentState.GearState = 0;
                }
            }
        }
        // DEBUG_PRINTLN("E: %d, T: %f, S: %f", Variable.CurrentState.EngineSpeed, Variable.CurrentState.EngineSpeedDifferential, Variable.CurrentState.Sensor);
        if (sensor_threshold_flag)
            SLEEP(1);
        else
            SLEEP(10);
    }
}

extern "C" void app_main()
{
    xTaskCreate(&mdns_task, "mdns_task", 4096, NULL, 0, NULL);
    xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);

    xTaskCreate(&queue_task, "queue_task", 4096, NULL, 0, NULL);

    xTaskCreate(&uart_task, "uart_task", 8192, NULL, 5, NULL);
    xTaskCreate(&bluetooth_acceptor_task, "bluetooth_acceptor_task", 8192, NULL, 5, NULL);
    // xTaskCreatePinnedToCore(&imu_task, "imu_task", 4096, NULL, 1, NULL, 0);

    xTaskCreate(&sensor_task, "sensor_task", 4096, NULL, 1, NULL);
    xTaskCreate(&adc_task, "adc_task", 4096, NULL, 1, NULL);
    xTaskCreate(&delta_task, "delta_task", 4096, NULL, 1, NULL);
    xTaskCreate(&main_task, "main_task", 4096, NULL, 1, NULL);
    wifi.init();
    // wifi.connect("Eddy_Wifi", "00000000", false);

    StoragePackets_t storage_packets = {.Operate = StoragePackets_t::Read};
    xQueueSend(StorageQueue, (void *)&storage_packets, 0);

    BlinkPackets_t blink_packets = {
        .StatePeriod = 10000,
        .BlinkPeriod = 200,
        .BlinkTime = 10,
        .BlinkNumber = 1,
    };
    xQueueSend(BlinkQueue, (void *)&blink_packets, 0);

    BuzzerPackets_t buzzer_packets = {
        .Frequency = BUZZER_DEFAULT_FREQUENCY,
        .Ring_Period = 40,
        .Ring_Encoded = 0b11111111,
    };
    xQueueSend(BuzzerQueue, (void *)&buzzer_packets, 0);
}