import serial.tools.list_ports

Import("env")



def detect_usb():
    """
    Detect if USB device (serial port) is connected. 
    """
    ports = list(serial.tools.list_ports.comports())
    result = False
    
    for port in ports:
        if "USB" in port.description:
            result = True
            break
    
    return result

def set_upload_method():
    if detect_usb():
        print("USB detected, setting upload protocol to serial")
        env.Replace(UPLOAD_PROTOCOL="esptool")
    else:
        print("No USB detected, using OTA")
        env.Replace(UPLOAD_PROTOCOL="espota", UPLOAD_PORT="your_device.local")

# Run the detection before the build
set_upload_method()