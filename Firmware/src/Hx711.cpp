#include "Hx711.h"
#include "esp32/rom/ets_sys.h"
#include "driver/gpio.h"
#include "Math.h"

void IRAM_ATTR doutPin_isr_handler(void *arg)
{
    Hx711 *instance = static_cast<Hx711 *>(arg);
    instance->read();
}

Hx711::Hx711(int clockPin, int doutPin, Gain gain)
    : clockPin(clockPin), doutPin(doutPin), gain(gain), offset(0), scale(1.0), averageNum(0), averageIndex(0), exceptionCounter(0)
{
    gpio_config_t clockPin_conf = {
        .pin_bit_mask = (1ULL << clockPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t doutPin_conf = {
        .pin_bit_mask = (1ULL << doutPin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL,
    };
    gpio_config(&clockPin_conf);
    gpio_config(&doutPin_conf);
    gpio_set_level((gpio_num_t)clockPin, 0);
}

Hx711::~Hx711()
{
    if (readbyISR_flag)
        readbyISR_stop();
    if (averageBuffer)
        free(averageBuffer);
}

void Hx711::start()
{
    gpio_set_level((gpio_num_t)clockPin, 0);
}
void Hx711::stop()
{
    gpio_set_level((gpio_num_t)clockPin, 0);
    gpio_set_level((gpio_num_t)clockPin, 1);
}

void Hx711::read()
{
    for (uint8_t add = 0; add < 3; add++)
    {
        uint8_t value = 0;
        for (int8_t i = 7; i >= 0; i--)
        {
            gpio_set_level((gpio_num_t)clockPin, 1);
            esp_rom_delay_us(4);
            value |= gpio_get_level((gpio_num_t)doutPin) << i;
            gpio_set_level((gpio_num_t)clockPin, 0);
            esp_rom_delay_us(5);
        }
        readBuffer[add] = value;
    }
    for (uint8_t i = 0; i < gain; i++)
    {
        gpio_set_level((gpio_num_t)clockPin, 1);
        esp_rom_delay_us(5);
        gpio_set_level((gpio_num_t)clockPin, 0);
        esp_rom_delay_us(5);
    }
    currentData = static_cast<long>((static_cast<unsigned long>((readBuffer[0] & 0x80) ? 0xFF : 0x00) << 24 |
                                     static_cast<unsigned long>(readBuffer[0]) << 16 |
                                     static_cast<unsigned long>(readBuffer[1]) << 8 |
                                     static_cast<unsigned long>(readBuffer[2])));
    if (averageNum != 0)
    {
        int64_t temp = 0;
        for (uint8_t i = 0; i < averageNum; i++)
            temp += averageBuffer[i];
        temp /= averageNum;
        if ((ABS(currentData) > ABS(temp) * 1.01 || ABS(currentData) < ABS(temp) * 0.99) && ++exceptionCounter < 5)
            return;
        exceptionCounter = 0;
        averageBuffer[averageIndex++] = currentData;

        if (averageIndex >= averageNum)
            averageIndex = 0;
    }
}
void Hx711::readbyISR()
{
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add((gpio_num_t)doutPin, doutPin_isr_handler, (void *)this);
    readbyISR_flag = true;
}

void Hx711::readbyISR_stop()
{
    gpio_isr_handler_remove((gpio_num_t)doutPin);
    gpio_uninstall_isr_service();
    readbyISR_flag = false;
}

long Hx711::GetRawData()
{
    return currentData;
}
float Hx711::GetConvertData()
{
    if (averageNum == 0)
        return (((currentData / (float)BASE_SCALE) + BASE_OFFSET) + offset) * scale;
    else
    {
        float temp = 0;
        for (int i = 0; i < averageNum; i++)
            temp += averageBuffer[i];
        temp = ((((temp / (float)averageNum) / (float)BASE_SCALE) + BASE_OFFSET) + offset) * scale;
        return temp;
    }
}
void Hx711::setAverage(uint8_t num)
{
    if (averageBuffer)
        free(averageBuffer);
    averageBuffer = (long *)calloc(num, sizeof(long));
    averageNum = num;
}
long Hx711::getAverage()
{
    float result = 0;
    for (int i = 0; i < averageNum; i++)
        result += averageBuffer[i];
    result /= averageNum;
    return (long)result;
}
void Hx711::setGain(Gain gain)
{
    this->gain = gain;
}
Gain Hx711::getGain()
{
    return gain;
}
void Hx711::setOffset(float offset)
{
    this->offset = offset;
}
float Hx711::getOffset()
{
    return offset;
}
void Hx711::setScale(float scale)
{
    this->scale = scale;
}
float Hx711::getScale()
{
    return scale;
}

void Hx711::readData()
{
    if (readbyISR_flag)
        return;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    while (true)
        if (gpio_get_level((gpio_num_t)doutPin) == 0)
            break;
    portENTER_CRITICAL(&mux);
    read();
    portEXIT_CRITICAL(&mux);
}
