#pragma once

#include <freertos/FreeRTOS.h>
#include "freertos/task.h"

#define BASE_OFFSET 280000
#define BASE_SCALE 10

typedef enum
{
    GAIN_128 = 1,
    GAIN_64 = 3,
    GAIN_32 = 2,
} Gain;

class Hx711
{
private:
    int clockPin;
    int doutPin;
    Gain gain;
    long offset;
    float scale;
    bool readbyISR_flag;
    uint8_t readBuffer[3];
    long currentData;
    long *averageBuffer;
    uint8_t averageNum;
    uint8_t averageIndex;

    uint8_t exceptionCounter;

    friend void IRAM_ATTR doutPin_isr_handler(void *arg);

public:
    Hx711(int clockPin, int doPin, Gain gain = GAIN_128);
    ~Hx711();
    void read();
    void start();
    void stop();
    void readbyISR();
    void readbyISR_stop();
    long GetRawData();
    float GetConvertData();

    void setAverage(uint8_t num);
    long getAverage();
    void setGain(Gain gain);
    Gain getGain();
    void setOffset(float offset);
    float getOffset();
    void setScale(float scale);
    float getScale();

    void readData();
};
