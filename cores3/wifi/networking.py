import serial
import json

ser = serial.Serial(
    port='/dev/ttyACM0',  # Find USB device using ls /dev
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

try:
    while True:
        if ser.in_waiting:
            line = ser.readline().strip()
            try:
                print(str(line))
            except json.JSONDecodeError:
                print("Invalid JSON received:", line)
except KeyboardInterrupt:
    ser.close()
    print("\nSerial connection closed.")
