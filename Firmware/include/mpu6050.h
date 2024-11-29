#pragma once

#include "Math.h"
#include "gen_i2c.h"

#define MPU6050_ADDRESS 0x68
#define MPU6050_PWR 0x6B
#define MPU6050_TEMP 0x41
#define MPU6050_Raw_ACC 0x3B
#define MPU6050_Raw_GYRO 0x43
#define MPU6050_WHO_AM_I 0x75

#define MPU6050_ACCEL_SCALE_CONFIG 0x1C
#define MPU6050_GYRO_SCALE_CONFIG 0x1B
#define MPU6050_SCALE_BIT 3

#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A

#define MPU6050_BANK_SEL 0x6D
#define MPU6050_MEM_START_ADDR 0x6E
#define MPU6050_MEM_R_W 0x6F

#define MPU6050_I2C_SLV0_ADDR 0x25
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT 1
#define MPU6050_USERCTRL_I2C_MST_EN_BIT 5

#define MPU6050_DMP_MEMORY_BANKS 8
#define MPU6050_DMP_MEMORY_BANK_SIZE 256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE 16

#define MPU6050_DMP_CFG_1 0x70
#define MPU6050_DMP_CFG_2 0x71

#define MPU6050_XG_OFFS_TC 0x00  //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_YG_OFFS_TC 0x01  //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_ZG_OFFS_TC 0x02  //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_X_FINE_GAIN 0x03 //[7:0] X_FINE_GAIN
#define MPU6050_Y_FINE_GAIN 0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_Z_FINE_GAIN 0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_XA_OFFS_H 0x06   //[15:0] XA_OFFS
#define MPU6050_XA_OFFS_L_TC 0x07
#define MPU6050_YA_OFFS_H 0x08 //[15:0] YA_OFFS
#define MPU6050_YA_OFFS_L_TC 0x09
#define MPU6050_ZA_OFFS_H 0x0A //[15:0] ZA_OFFS
#define MPU6050_ZA_OFFS_L_TC 0x0B
#define MPU6050_XG_OFFS_USRH 0x13 //[15:0] XG_OFFS_USR
#define MPU6050_XG_OFFS_USRL 0x14
#define MPU6050_YG_OFFS_USRH 0x15 //[15:0] YG_OFFS_USR
#define MPU6050_YG_OFFS_USRL 0x16
#define MPU6050_ZG_OFFS_USRH 0x17 //[15:0] ZG_OFFS_USR

#define MPU6050_TC_OFFSET_BIT 6
#define MPU6050_TC_OFFSET_LENGTH 6

#define MPU6050_TC_OTP_BNK_VLD_BIT 0

#define MPU6050_USER_CTRL 0x6A
#define MPU6050_USERCTRL_DMP_EN_BIT 7
#define MPU6050_USERCTRL_FIFO_EN_BIT 6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT 5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT 4
#define MPU6050_USERCTRL_DMP_RESET_BIT 3
#define MPU6050_USERCTRL_FIFO_RESET_BIT 2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT 1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT 0

#define MPU6050_INT_ENABLE 0x38
#define MPU6050_DMP_INT_STATUS 0x39
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_FIFO_EN 0x23
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_FIFO_COUNT 0x72
#define MPU6050_FIFO_R_W 0x74
#define MPU6050_DMP_ENABLE 0x80

#define MPU6050_PWR_MGMT_1 0x6B

#define MPU6050_TEMP_FIFO_EN_BIT 7
#define MPU6050_XG_FIFO_EN_BIT 6
#define MPU6050_YG_FIFO_EN_BIT 5
#define MPU6050_ZG_FIFO_EN_BIT 4
#define MPU6050_ACCEL_FIFO_EN_BIT 3
#define MPU6050_SLV2_FIFO_EN_BIT 2
#define MPU6050_SLV1_FIFO_EN_BIT 1
#define MPU6050_SLV0_FIFO_EN_BIT 0

typedef enum
{
    SUCCESS,
    CONNECTION_FAILED,
    SETTING_FAILED,
    WRITEMEMORY_FAILED,
} MPU6050_ERROR;

typedef enum
{
    MPU6050_CLOCK_INTERNAL = 0x00,
    MPU6050_CLOCK_PLL_XGYRO = 0x01,
    MPU6050_CLOCK_PLL_YGYRO = 0x02,
    MPU6050_CLOCK_PLL_ZGYRO = 0x03,
    MPU6050_CLOCK_PLL_EXT32K = 0x04,
    MPU6050_CLOCK_PLL_EXT19M = 0x05,
    MPU6050_CLOCK_KEEP_RESET = 0x07,
} CLOCK_SOURCE;
typedef enum
{
    ACCEL_SCALE_2G = 0x00,
    ACCEL_SCALE_4G = 0x01,
    ACCEL_SCALE_8G = 0x02,
    ACCEL_SCALE_16G = 0x03,
} ACCEL_SCALE;
typedef enum
{
    GYRO_SCALE_250 = 0x00,
    GYRO_SCALE_500 = 0x01,
    GYRO_SCALE_1000 = 0x02,
    GYRO_SCALE_2000 = 0x03,
} GYRO_SCALE;

typedef enum
{
    DLPF_260_256 = 0x00,
    DLPF_184_188 = 0x01,
    DLPF_94_98 = 0x02,
    DLPF_44_42 = 0x03,
    DLPF_21_20 = 0x04,
    DLPF_10_10 = 0x05,
    DLPF_5_5 = 0x06,
} DLPF_MODE;

class mpu6050
{
private:
    gen_i2c *i2c;
    uint8_t abuffer[6];
    uint8_t bbuffer[6];

public:
    mpu6050(/* args */);
    ~mpu6050();
    MPU6050_ERROR initMPU6050(int sda_pin, int scl_pin, int i2c_freq_hz);
    uint8_t getRevision();

    void setMemoryStartAddress(uint8_t bank, bool prefetchEnabled, bool userBank, uint8_t startAddress);

    MPU6050_ERROR setScaleAccelRange(ACCEL_SCALE range);
    MPU6050_ERROR setScaleGyroRange(GYRO_SCALE range);

    MPU6050_ERROR setClockSource(CLOCK_SOURCE source);
    /// @brief
    /// @param rate SampleRate Freq = 1KHz / (rate + 1)
    /// @return ERROR Code
    MPU6050_ERROR setSampleRate(uint8_t rate);

    MPU6050_ERROR setDLPFMode(DLPF_MODE mode);

    void setSlaveAddress(uint8_t num, uint8_t address);
    void setI2CMasterModeEnabled(bool enabled);
    void resetI2CMaster();
    MPU6050_ERROR writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem);

    void setDMPConfig1(uint8_t config);
    void setDMPConfig2(uint8_t config);
    void setOTPBankValid(bool enabled);
    void setDMPEnabled(bool enabled);
    void resetDMP();
    void setFIFOEnabled(bool enabled);
    void resetFIFO();
    int16_t getFIFOCount();
    uint8_t dmpReadFIFOPacket(FIFOPacket *packet);

    void setAccOffset(int8_t xOffset, int8_t yOffset, int8_t zOffset);
    void setGyroOffset(int8_t xOffset, int8_t yOffset, int8_t zOffset);

    void calibrateGyroOffset(uint64_t calibrateNum, Vec3i *offset);

    uint8_t getDeviceID();
    float getTemp();
    void getRawAcc(Vec3i *data);
    void getRawGyro(Vec3i *data);

    void convertToQuate(const Vec4i *sourceData, Vec4f *targetData);
    void convertToAcc(const Vec3i *sourceData, Vec3f *targetData);
    void convertToGyro(const Vec3i *sourceData, Vec3f *targetData);

    void calibration();

    void calibrateAccel(uint8_t Loops);
    void calibrateGyro(uint8_t Loops);
    void calibratePID(uint8_t ReadAddress, float kP, float kI, uint8_t Loops);
};
