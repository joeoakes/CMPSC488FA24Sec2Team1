import serial
import json

ser = serial.Serial(
    port='/dev/SOMETHING',  # Gotta figure out what it is
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

try:
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            try:
                data = json.loads(line)
                print(data)
            except json.JSONDecodeError:
                print("Invalid JSON received:", line)
except KeyboardInterrupt:
    ser.close()
    print("\nSerial connection closed.")