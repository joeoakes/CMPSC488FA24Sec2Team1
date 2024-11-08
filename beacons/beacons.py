import numpy as np
import asyncio
from bleak import BleakScanner

def rssi_to_distance(rssi, reference_rssi=-60, path_loss_exponent=2):
    """Convert RSSI to distance using the log-distance path loss model."""
    return 10 ** ((reference_rssi - rssi) / (10 * path_loss_exponent))

def trilaterate(beacons, distances):
    """Perform trilateration to find the (x, y) position of the device."""
    x1, y1 = beacons[0]
    x2, y2 = beacons[1]
    x3, y3 = beacons[2]

    d1, d2, d3 = distances

    # Set up the equations
    A = 2*x2 - 2*x1
    B = 2*y2 - 2*y1
    C = d1**2 - d2**2 - x1**2 + x2**2 - y1**2 + y2**2
    D = 2*x3 - 2*x2
    E = 2*y3 - 2*y2
    F = d2**2 - d3**2 - x2**2 + x3**2 - y2**2 + y3**2

    # Calculate the coordinates using linear equations
    x = (C - (B/F)*E) / (A - (B/F)*D)
    if B != 0:
        y = (C - A*x) / B
    else:
        y = (F - D*x) / E

    return x, y

# Beacon Positions
beacons = [
    (0, 0),  # Beacon A at (0, 0)
    (10, 0), # Beacon B at (10, 0)
    (5, 10)  # Beacon C at (5, 10)
]

async def main():
    BEACON_ADDRS = ["DC:0D:30:14:2F:00", "DC:0D:30:14:2F:26", "DC:0D:30:14:2F:2C"]
   
    while(True):
        distances = []
        devices = await BleakScanner.discover()
        for dev in devices:
            if dev.address not in BEACON_ADDRS:
                continue
            distances.append(rssi_to_distance(dev.rssi)) 
            print(dev.rssi, end=" ")
        print()
        print(distances)

        device_position = trilaterate(beacons, distances)
        print(f"Estimated device position: {device_position}")
     
asyncio.run(main())

