import os, sys, io
import M5
from M5 import *
from hardware import *
from unit import UltrasoundI2CUnit
import time
from math import atan2, degrees

# Group related global variables
# I2C and Sensors
i2c0 = None
ultrasonic_0 = None
adc6 = None
calibration_factor = 3.3 / 4095

# Magnetometer calibration variables
offset_x, offset_y, offset_z = 0, 0, 0
scale_x, scale_y, scale_z = 1, 1, 1

# UI Labels (grouped in a dictionary for better management)
labels = {
    'depth': None,
    'voltage': None,
    'xyz': [],
    'accel': [],
    'gyro': [],
    'mag': [],
    'heading': None
}

def calibrate_magnetometer():
    """Calibrate the magnetometer and calculate offsets and scale factors"""
    global offset_x, offset_y, offset_z, scale_x, scale_y, scale_z
    
    # Clear screen and create calibration labels
    Widgets.fillScreen(0x222222)
    status_label = Widgets.Label("Calibrating Magnetometer...", 1, 5, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18)
    instruction_label = Widgets.Label("Rotate device in all directions", 1, 25, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18)
    time_label = Widgets.Label("Time remaining: 30s", 1, 45, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18)
    min_label = Widgets.Label("Min:", 1, 65, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18)
    max_label = Widgets.Label("Max:", 1, 85, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18)
    
    min_x, min_y, min_z = 32767, 32767, 32767
    max_x, max_y, max_z = -32768, -32768, -32768
    
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < 30000:  # 30 seconds calibration
        mag_x, mag_y, mag_z = Imu.getMag()
        
        min_x, min_y, min_z = min(min_x, mag_x), min(min_y, mag_y), min(min_z, mag_z)
        max_x, max_y, max_z = max(max_x, mag_x), max(max_y, mag_y), max(max_z, mag_z)
        
        # Update display
        remaining = 30 - time.ticks_diff(time.ticks_ms(), start_time) // 1000
        time_label.setText(f"Time remaining: {remaining}s")
        min_label.setText(f"Min: X:{min_x:.1f} Y:{min_y:.1f} Z:{min_z:.1f}")
        max_label.setText(f"Max: X:{max_x:.1f} Y:{max_y:.1f} Z:{max_z:.1f}")
        
        time.sleep(0.1)
    
    # Calculate offsets and scale factors
    offset_x = (max_x + min_x) / 2
    offset_y = (max_y + min_y) / 2
    offset_z = (max_z + min_z) / 2
    
    scale_x = 2 / (max_x - min_x)
    scale_y = 2 / (max_y - min_y)
    scale_z = 2 / (max_z - min_z)
    
    # Show results
    status_label.setText("Calibration Complete!")
    instruction_label.setText("Results:")
    time_label.setText(f"Offsets: X:{offset_x:.1f} Y:{offset_y:.1f} Z:{offset_z:.1f}")
    min_label.setText(f"Scales: X:{scale_x:.2f} Y:{scale_y:.2f} Z:{scale_z:.2f}")
    max_label.setText("Starting main program...")
    
    time.sleep(2)  # Show results for 2 seconds

def calibrate():
    """Calibrate all sensors"""
    try:
        # Magnetometer calibration
        calibrate_magnetometer()
        return True
    except Exception as e:
        print(f"Calibration failed: {e}")
        return False

def setup():
    global labels
    global i2c0, ultrasonic_0, adc6

    M5.begin()

    # Ultrasonic
    i2c0 = I2C(0, scl=Pin(1), sda=Pin(2), freq=100000)
    ultrasonic_0 = UltrasoundI2CUnit(i2c0)
    adc6 = ADC(Pin(10), atten=ADC.ATTN_11DB)

    # Display
    Widgets.fillScreen(0x222222)
    
    labels['depth'] = Widgets.Label("Depth:", 1, 5, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18)
    labels['voltage'] = Widgets.Label("Voltage:", 1, 25, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18)
    labels['heading'] = Widgets.Label("Degrees:", 1, 130, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18)
    # X Y Z
    labels['xyz'].append(Widgets.Label("X ", 80, 45, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    labels['xyz'].append(Widgets.Label("Y ", 160, 45, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    labels['xyz'].append(Widgets.Label("Z ", 230, 45, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    # Titles
    labels['accel'].append(Widgets.Label("Accel: ", 1, 70, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    labels['gyro'].append(Widgets.Label("Gyro: ", 1, 90, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    labels['mag'].append(Widgets.Label("Mag: ", 1, 110, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    # Accel Data
    labels['accel'].append(Widgets.Label("", 80, 70, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    labels['accel'].append(Widgets.Label("", 160, 70, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    labels['accel'].append(Widgets.Label("", 230, 70, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    # Gyro Data
    labels['gyro'].append(Widgets.Label("", 80, 90, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    labels['gyro'].append(Widgets.Label("", 160, 90, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    labels['gyro'].append(Widgets.Label("", 230, 90, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    # Mag Data
    labels['mag'].append(Widgets.Label("", 80, 110, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    labels['mag'].append(Widgets.Label("", 160, 110, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    labels['mag'].append(Widgets.Label("", 230, 110, 1.0, 0xFFFFFF, 0x222222, Widgets.FONTS.DejaVu18))
    
def loop():
    try:
        M5.update()
        
        # Get sensor readings
        accel = Imu.getAccel()
        gyro = Imu.getGyro()
        mag = Imu.getMag()
        
        # Apply magnetometer calibration
        mag_x = (mag[0] - offset_x) * scale_x
        mag_y = (mag[1] - offset_y) * scale_y
        mag_z = (mag[2] - offset_z) * scale_z
        calibrated_mag = (mag_x, mag_y, mag_z)
        
        # Calculate heading using X and Z instead of X and Y
        heading_radians = atan2(mag_z, mag_x)
        heading_degrees = degrees(heading_radians)
        if heading_degrees < 0:
            heading_degrees += 360
        
        distance = ultrasonic_0.get_target_distance(1)
        adc_value = adc6.read()
        voltage = adc_value * calibration_factor

        # Update display
        labels['depth'].setText(f"Depth {distance:.2f} mm")
        labels['voltage'].setText(f"Voltage: {voltage:.2f} V")
        labels['heading'].setText(f"Degrees: {heading_degrees:.1f}°")
        
        # Update sensor data displays using calibrated magnetometer values
        for i, val in enumerate(accel):
            labels['accel'][i+1].setText(f"{val:.2f}")
        for i, val in enumerate(gyro):
            labels['gyro'][i+1].setText(f"{val:.2f}")
        for i, val in enumerate(calibrated_mag):
            labels['mag'][i+1].setText(f"{val:.2f}")
        
        # Log data in JSON-like format
        data = {
            'accel': [round(x, 2) for x in accel],
            'gyro': [round(x, 2) for x in gyro],
            'mag': [round(x, 2) for x in calibrated_mag],
            'heading': round(heading_degrees, 2),
            'ultrasonic': round(distance, 2),
            'solar': round(voltage, 2)
        }
        print(data)
        
    except Exception as e:
        print(f"Error in loop: {e}")

if __name__ == "__main__":
    try:
        calibrate()
        setup()
        while True:
            loop()
    except (Exception, KeyboardInterrupt) as e:
        try:
            from utility import print_error_msg
            print_error_msg(e)
        except ImportError:
            print("please update to latest firmware")
