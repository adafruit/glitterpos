import time
import math
import board
import busio
import adafruit_lsm9ds1

i2c = busio.I2C(board.SCL, board.SDA)
compass = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

mag_min = [-0.10206, 0.00238, -0.10458]
mag_max = [0.44926, 0.56938, 0.4501]

def calibrate_compass(compass):
    mag_min = [1000, 1000, 1000]
    mag_max = [-1000, -1000, -1000]
    print("Magnetometer Calibration")
    lastDisplayTime = time.monotonic()
    while True:
        x, y, z = compass.magnetometer
        mag_vals = [x, y, z]
        for i in range(3):
            mag_min[i] = min(mag_min[i], mag_vals[i])
            mag_max[i] = max(mag_max[i], mag_vals[i])
        # display once every two seconds
        if (time.monotonic() - lastDisplayTime >= 1):
            print("Mag Min: ", mag_min, "& Max:", mag_max)
            lastDisplayTime = time.monotonic();

#calibrate_compass(compass)

def map_range(x, in_min, in_max, out_min, out_max):
    mapped = (x-in_min) * (out_max - out_min) / (in_max-in_min) + out_min
    if out_min <= out_max:
        return max(min(mapped, out_max), out_min)
    return min(max(mapped, out_max), out_min)

while True:
    x, y, z = compass.magnetometer
    print("Uncalibrated: ", x, y, z, end="")
    x = map_range(x, mag_min[0], mag_max[0], -1, 1)
    y = map_range(y, mag_min[1], mag_max[1], -1, 1)
    z = map_range(z, mag_min[2], mag_max[2], -1, 1)
    print(" \tCalibrated: ", x, y, z)
    
    compass_heading = (math.atan2(y, x) * 180) / math.pi
    if compass_heading < 0:
        compass_heading += 360
    print("Heading: ", compass_heading)
    time.sleep(0.5)