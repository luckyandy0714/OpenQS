#include "Variable.h"
#include <string.h>

class QueueVector
{
private:
    char *buffer;
    size_t buffer_size;
    size_t tail;

public:
    QueueVector(const size_t buffer_size);
    ~QueueVector();
    size_t size();
    bool add(char src);
    bool add(const void *src, const size_t size);
    bool remove(const size_t offset, const size_t length);
    void *data();
    void clear();

    int get(const size_t index);
    int get(const size_t index, void *dst, const size_t len);
    void set(const size_t index, const char src);

    int find(void *searchArray, const size_t size);
};

QueueVector::QueueVector(const size_t buffer_size)
{
    this->buffer_size = buffer_size;
    buffer = (char *)malloc(buffer_size);
    tail = 0;
}

QueueVector::~QueueVector()
{
    if (buffer)
        free(buffer);
}

inline size_t QueueVector::size()
{
    return tail;
}

bool QueueVector::add(char src)
{
    if (tail < buffer_size)
    {
        buffer[tail] = src;
        tail++;
        return true;
    }
    return false;
}

bool QueueVector::add(const void *src, const size_t len)
{
    if (!src || len == 0)
        return false;
    if (tail + len > buffer_size)
        return false;
    memcpy(buffer + tail, src, len);
    tail += len;
    return true;
}
bool QueueVector::remove(const size_t offset, const size_t len)
{
    if (offset + len > tail)
        return false;
    memmove(buffer + offset, buffer + offset + len, tail - (offset + len));
    tail -= len;
    return true;
}
inline void *QueueVector::data()
{
    return (void *)(buffer);
}
void QueueVector::clear()
{
    tail = 0;
}
inline int QueueVector::get(const size_t index)
{
    if (index < tail)
        return buffer[index];
    else
        return -1;
}
inline int QueueVector::get(const size_t index, void *dst, const size_t len)
{
    if (index >= tail || index + len >= tail)
        return -1;
    memcpy(dst, buffer + index, len);
    return len;
}
inline void QueueVector::set(const size_t index, const char src)
{
    if (index < tail)
        ((char *)buffer)[index] = src;
}
int QueueVector::find(void *searchArray, const size_t size)
{
    if (size <= 0)
        return -1;
    for (size_t i = 0; i < tail; i++)
        for (size_t j = 0; j < size; j++)
            if (buffer[i] == ((char *)searchArray)[j])
                return i;
    return -1;
}

QueueVector ParseCommand_Buffer = QueueVector(BLUETOOTH_COMMAND_BUFFER_SIZE);
static char find_command_array[]{
    BLUETOOTH_HANDSHAKE_COMMAND,
    BLUETOOTH_READ_ADDRESS_COMMAND,
    BLUETOOTH_WRITE_ADDRESS_COMMAND,
    BLUETOOTH_FUNCTION_COMMAND,
    BLUETOOTH_CUTOFF_LOG_COMMAND,
};
ParseCommand::ReceiveCommandCallback ParseCommand::receiveCommandCallback = nullptr;
ParseCommand::TransmitCommandCallback ParseCommand::transmitCommandCallback = nullptr;
ParseCommand::WriteAddressCommandCallback ParseCommand::writeAddressCommandCallback = nullptr;

ParseCommand::EnableQSCommandCallback ParseCommand::enableQSCommandCallback = nullptr;
ParseCommand::ResetCommandCallback ParseCommand::resetCommandCallback = nullptr;
ParseCommand::ResetDefaultVariableCommandCallback ParseCommand::resetDefaultVariableCommandCallback = nullptr;
ParseCommand::ResetSensorOffsetCommandCallback ParseCommand::resetSensorOffsetCommandCallback = nullptr;
ParseCommand::ManualTriggerCommandCallback ParseCommand::manualTriggerCommandCallback = nullptr;
ParseCommand::SetWifiCommandCallback ParseCommand::setWifiCommandCallback = nullptr;

char *ParseCommand::variable_ptr = nullptr;

void ParseCommand::setVariable_ptr(const void *ptr)
{
    variable_ptr = (char *)ptr;
}
void ParseCommand::setReceiveCallback(ReceiveCommandCallback callback)
{
    receiveCommandCallback = callback;
}
void ParseCommand::setTransmitCallback(TransmitCommandCallback callback)
{
    transmitCommandCallback = callback;
}
void ParseCommand::setWriteAddressCommandCallback(WriteAddressCommandCallback callback)
{
    writeAddressCommandCallback = callback;
}
void ParseCommand::setEnableQSCommandCallback(EnableQSCommandCallback callback)
{
    enableQSCommandCallback = callback;
}
void ParseCommand::setResetCommandCallback(ResetCommandCallback callback)
{
    resetCommandCallback = callback;
}
void ParseCommand::setResetDefaultVariableCommandCallback(ResetDefaultVariableCommandCallback callback)
{
    resetDefaultVariableCommandCallback = callback;
}
void ParseCommand::setResetSensorOffsetCommandCallback(ResetSensorOffsetCommandCallback callback)
{
    resetSensorOffsetCommandCallback = callback;
}
void ParseCommand::setManualTriggerCommandCallbackCallback(ManualTriggerCommandCallback callback)
{
    manualTriggerCommandCallback = callback;
}
void ParseCommand::setSetWifiCommandCallbackCallback(SetWifiCommandCallback callback)
{
    setWifiCommandCallback = callback;
}
bool ParseCommand::parseCommand(const void *src, const size_t len)
{
    if (src && len != 0)
        ParseCommand_Buffer.add(src, len);
    int first = ParseCommand_Buffer.get(0);

    if (first == -1)
        return false;
    switch (first)
    {
    case BLUETOOTH_HANDSHAKE_COMMAND:
        ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
        break;
    case BLUETOOTH_READ_ADDRESS_COMMAND:
    {
        if (ParseCommand_Buffer.size() < 4)
            break;
        const int address = ParseCommand_Buffer.get(1);
        const int length = ParseCommand_Buffer.get(2);
        if (ParseCommand_Buffer.get(3) != BLUETOOTH_READ_ADDRESS_COMMAND || length > sizeof(Variable) - address) // verify Command Check bit
        {
            ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
            break;
        }
        ParseCommand_Buffer.remove(0, 4); // Removed parse data size.

        QueueVector read_transmitter(4 + length);
        read_transmitter.add(BLUETOOTH_READ_ADDRESS_COMMAND); // Fill in the first byte of BLUETOOTH_READ_ADDRESS_COMMAND
        read_transmitter.add(address);                        // Fill in the byte of address
        read_transmitter.add(length);                         // Fill in the byte of length
        read_transmitter.add(variable_ptr + address, length); // Fill in memory data from address
        read_transmitter.add(BLUETOOTH_READ_ADDRESS_COMMAND); // Fill in the last byte of BLUETOOTH_READ_ADDRESS_COMMAND

        if (transmitCommandCallback)
            transmitCommandCallback(read_transmitter.data(), read_transmitter.size());
    }
    break;
    case BLUETOOTH_WRITE_ADDRESS_COMMAND:
    {
        if (ParseCommand_Buffer.size() < 3)
            break;
        const int address = ParseCommand_Buffer.get(1);
        const int length = ParseCommand_Buffer.get(2);
        if (ParseCommand_Buffer.size() < 4 + length)
            break;

        if (ParseCommand_Buffer.get(3 + length) != BLUETOOTH_WRITE_ADDRESS_COMMAND || length > sizeof(Variable) - address)
        {
            ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
            break;
        }
        ParseCommand_Buffer.get(3, variable_ptr + address, length); // Fill in variable from memory data
        if (writeAddressCommandCallback)
            writeAddressCommandCallback();

        ParseCommand_Buffer.remove(0, 4 + length); // Removed parse data size.
        QueueVector write_transmitter(4);
        write_transmitter.add(BLUETOOTH_WRITE_ADDRESS_COMMAND); // Fill in the first byte of BLUETOOTH_WRITE_ADDRESS_COMMAND
        write_transmitter.add(address);                         // Fill in the byte of address
        write_transmitter.add(length);                          // Fill in the byte of length
        write_transmitter.add(BLUETOOTH_WRITE_ADDRESS_COMMAND); // Fill in the last byte of BLUETOOTH_WRITE_ADDRESS_COMMAND

        if (transmitCommandCallback)
            transmitCommandCallback(write_transmitter.data(), write_transmitter.size());
    }
    break;
    case BLUETOOTH_FUNCTION_COMMAND:
    {
        if (ParseCommand_Buffer.size() < 3)
            break;
        const int f_enum = ParseCommand_Buffer.get(1);
        const int length = ParseCommand_Buffer.get(2);
        if (ParseCommand_Buffer.size() < 4 + length)
            break;

        if (ParseCommand_Buffer.get(3 + length) != BLUETOOTH_FUNCTION_COMMAND)
        {
            ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
            break;
        }
        switch (f_enum)
        {
        case EnableQS:
            if (length != 1)
            {
                ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
                break;
            }
            if (enableQSCommandCallback)
                enableQSCommandCallback(ParseCommand_Buffer.get(3) != 0);
            break;
        case Restart:
            if (length != 0)
            {
                ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
                break;
            }
            if (resetCommandCallback)
                resetCommandCallback();
            break;
        case ResetDefaultVariable:
            if (length != 0)
            {
                ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
                break;
            }
            if (resetDefaultVariableCommandCallback)
                resetDefaultVariableCommandCallback();
            break;
        case Reset_Sensor_Offset:
            if (length != 0)
            {
                ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
                break;
            }
            if (resetSensorOffsetCommandCallback)
                resetSensorOffsetCommandCallback();
            break;
        case ManualTrigger:
            if (length != 2)
            {
                ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
                break;
            }
            {
                uint16_t value = 0;
                ((char *)(&value))[0] = ParseCommand_Buffer.get(3);
                ((char *)(&value))[1] = ParseCommand_Buffer.get(4);
                if (manualTriggerCommandCallback)
                    manualTriggerCommandCallback(value);
            }
            break;
        case Set_Wifi:
            ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
            if (setWifiCommandCallback)
                setWifiCommandCallback(NULL, NULL);
            break;
        default:
            ParseCommand_Buffer.set(0, BLUETOOTH_SKIP_COMMAND);
            break;
        }
        if (ParseCommand_Buffer.get(0) != BLUETOOTH_SKIP_COMMAND)
        {
            ParseCommand_Buffer.remove(0, 4 + length); // Removed parse data size.
            QueueVector write_transmitter(4);
            write_transmitter.add(BLUETOOTH_FUNCTION_COMMAND); // Fill in the first byte of BLUETOOTH_WRITE_ADDRESS_COMMAND
            write_transmitter.add(f_enum);                     // Fill in the byte of enum
            write_transmitter.add(length);                     // Fill in the byte of length
            write_transmitter.add(BLUETOOTH_FUNCTION_COMMAND); // Fill in the last byte of BLUETOOTH_WRITE_ADDRESS_COMMAND
            if (transmitCommandCallback)
                transmitCommandCallback(write_transmitter.data(), write_transmitter.size());
        }
    }
    break;
    default:
    {
        const int available_command_index = ParseCommand_Buffer.find(find_command_array, sizeof(find_command_array));
        if (available_command_index == -1)
            ParseCommand_Buffer.clear();
        else
            ParseCommand_Buffer.remove(0, available_command_index);
    }
    break;
    }
    return true;
}

void ParseCommand::transmit_cutoff_log_Command(CUTOFF_LOG cutoff_log)
{
    QueueVector write_transmitter(sizeof(CUTOFF_LOG) + 4);
    write_transmitter.add(BLUETOOTH_CUTOFF_LOG_COMMAND);
    write_transmitter.add(0);
    write_transmitter.add(sizeof(CUTOFF_LOG));
    write_transmitter.add(&cutoff_log, sizeof(CUTOFF_LOG));
    write_transmitter.add(BLUETOOTH_CUTOFF_LOG_COMMAND);
    if (transmitCommandCallback)
        transmitCommandCallback(write_transmitter.data(), write_transmitter.size());
}

void ParseCommand::clear()
{
    ParseCommand_Buffer.clear();
}
