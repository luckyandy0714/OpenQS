#pragma once
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define BLUETOOTH_COMMAND_BUFFER_SIZE 512
#define BLUETOOTH_RECEIVER_TIMEOUT 100

#define BLUETOOTH_HANDSHAKE_COMMAND 0xFB
#define BLUETOOTH_READ_ADDRESS_COMMAND 0xFC
#define BLUETOOTH_WRITE_ADDRESS_COMMAND 0xFD
#define BLUETOOTH_SKIP_COMMAND 0xFE

#define BLUETOOTH_FUNCTION_COMMAND 0xF9
#define BLUETOOTH_CUTOFF_LOG_COMMAND 0xF8

// BLUETOOTH_COMMAND:
// Handshake Receiver(Host To MCU) Commend:    {BLUETOOTH_HANDSHAKE_COMMAND | ADDRESS | SIZE | BLUETOOTH_HANDSHAKE_COMMAND}
// Handshake Transmitter(MCU To Host) Commend: {BLUETOOTH_HANDSHAKE_COMMAND | ADDRESS | SIZE | DATA[0] | ... | DATA[SIZE] | BLUETOOTH_HANDSHAKE_COMMAND}
// Read Receiver(Host To MCU) Commend:         {BLUETOOTH_READ_ADDRESS_COMMAND | ADDRESS | SIZE | BLUETOOTH_READ_ADDRESS_COMMAND}
// Read Transmitter(MCU To Host) Commend:      {BLUETOOTH_READ_ADDRESS_COMMAND | ADDRESS | SIZE | DATA[0] | ... | DATA[SIZE] | BLUETOOTH_READ_ADDRESS_COMMAND}
// Write Receiver(Host To MCU) Commend:        {BLUETOOTH_WRITE_ADDRESS_COMMAND | ADDRESS | SIZE | DATA[0] | ... | DATA[SIZE] | BLUETOOTH_WRITE_ADDRESS_COMMAND}
// Write Transmitter(MCU To Host) Commend:     {BLUETOOTH_WRITE_ADDRESS_COMMAND | ADDRESS | SIZE | BLUETOOTH_WRITE_ADDRESS_COMMAND}

// Function Receiver(Host To MCU) Commend:    {BLUETOOTH_FUNCTION_COMMAND | ENUM | SIZE | DATA[0] | ... | DATA[SIZE] | BLUETOOTH_FUNCTION_COMMAND}
// Function Transmitter(MCU To Host) Commend: {BLUETOOTH_FUNCTION_COMMAND | ENUM | SIZE | BLUETOOTH_FUNCTION_COMMAND}

// CutOff Log Transmitter(MCU To Host) Commend: {BLUETOOTH_CUTOFF_LOG_COMMAND | 0 | SIZE | DATA[0] | ... | DATA[SIZE] | BLUETOOTH_CUTOFF_LOG_COMMAND}
#pragma pack(push, 1) // Turn off memory alignment
typedef struct
{
    struct CURRENTSTATE
    {
        float EngineSpeed;
        float EngineSpeed_Delta;
        uint8_t GearState;
        uint8_t a;
        struct
        {
            float W;
            float X;
            float Y;
            float Z;
        } Quate;
        float Sensor;
        float Temperature;
        float Voltage;
    } CurrentState;
    struct
    {
        bool CUTOFF_Positive_Enable : 1;
        bool CUTOFF_Negative_Enable : 1;
        bool CUTOFF_LimitOfMinGear : 1;
        bool CUTOFF_LimitOfMaxGear : 1;
        bool CUTOFF_Smooth_Curve : 1;
        bool EngineSpeed_Ratio : 1;
        bool EngineSpeed_Delta_Ratio : 1;
        bool GearState_Ratio : 1;
        bool EngineSpeed_Half : 1;
        bool Sensor_Direction_Reverse : 1;
        bool Silent_Mode : 1;
        bool EngineStart_Delay : 1;
        bool IMU_Enable : 1;
        bool b : 1;
        bool c : 1;
        bool d : 1;
    } BitConfig;
    struct
    {
        uint8_t CUTOFF_Time;
        uint8_t CUTOFF_BeforeDelayTime;
        uint8_t CUTOFF_AfterDelayTime;
        uint8_t e;
        float CUTOFF_SmoothTime_T1;
        float CUTOFF_SmoothTime_T2;
        float Sensor_Positive_Threshold;
        float Sensor_Negative_Threshold;
        float Sensor_Offset;
        float Sensor_Scale;
        uint16_t EngineSpeed_Ratio_Map_Label[2][ENGINESPEED_RATIO_MAP_SIZE];
        uint8_t EngineSpeed_Ratio_Map[2][ENGINESPEED_RATIO_MAP_SIZE];
        int16_t EngineSpeed_Delta_Ratio_Map_Label[2][ENGINESPEED_DELTA_RATIO_MAP_SIZE];
        uint8_t EngineSpeed_Delta_Ratio_Map[2][ENGINESPEED_DELTA_RATIO_MAP_SIZE];
        uint8_t GearState_Ratio_Map[2][GEARSTATE_NUMBER - 1];
    } ValueConfig;
} Variable;

typedef struct
{
    float EngineSpeed;
    float EngineSpeed_Delta;
    uint8_t GearState;
    uint16_t CutOff_Time;

} CUTOFF_LOG;

#pragma pack(pop) // Revert to default memory alignment

typedef enum
{
    EnableQS,
    Restart,
    ResetDefaultVariable,
    Reset_Sensor_Offset,
    ManualTrigger,
    Set_Wifi,
} BLUETOOTH_FUNCTION;

class ParseCommand
{
public:
    typedef void (*ReceiveCommandCallback)(void *, const size_t);
    typedef void (*TransmitCommandCallback)(const void *, const size_t);
    typedef void (*WriteAddressCommandCallback)(void);

    typedef void (*EnableQSCommandCallback)(bool);
    typedef void (*ResetCommandCallback)(void);
    typedef void (*ResetDefaultVariableCommandCallback)(void);
    typedef void (*ResetSensorOffsetCommandCallback)(void);
    typedef void (*ManualTriggerCommandCallback)(const uint16_t);
    typedef void (*SetWifiCommandCallback)(const char *, const char *);

    static void setVariable_ptr(const void *ptr);
    static void setReceiveCallback(ReceiveCommandCallback callback);
    static void setTransmitCallback(TransmitCommandCallback callback);
    static void setWriteAddressCommandCallback(WriteAddressCommandCallback callback);

    static void setEnableQSCommandCallback(EnableQSCommandCallback callback);
    static void setResetCommandCallback(ResetCommandCallback callback);
    static void setResetDefaultVariableCommandCallback(ResetDefaultVariableCommandCallback callback);
    static void setResetSensorOffsetCommandCallback(ResetSensorOffsetCommandCallback callback);
    static void setManualTriggerCommandCallbackCallback(ManualTriggerCommandCallback callback);
    static void setSetWifiCommandCallbackCallback(SetWifiCommandCallback callback);

    static bool parseCommand(const void *src, const size_t len);
    static void transmit_cutoff_log_Command(CUTOFF_LOG cutoff_log);
    static void clear();

private:
    static char *variable_ptr;

    static ReceiveCommandCallback receiveCommandCallback;
    static TransmitCommandCallback transmitCommandCallback;
    static WriteAddressCommandCallback writeAddressCommandCallback;

    static EnableQSCommandCallback enableQSCommandCallback;
    static ResetCommandCallback resetCommandCallback;
    static ResetDefaultVariableCommandCallback resetDefaultVariableCommandCallback;
    static ResetSensorOffsetCommandCallback resetSensorOffsetCommandCallback;
    static ManualTriggerCommandCallback manualTriggerCommandCallback;
    static SetWifiCommandCallback setWifiCommandCallback;
};
