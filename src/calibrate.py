import time
import math
import board
import busio
import adafruit_lsm9ds1
from glitterpos_util import map_range

i2c = busio.I2C(board.SCL, board.SDA)
compass = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

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

        # Display once every three seconds:
        if (time.monotonic() - lastDisplayTime >= 3):
            print("Uncalibrated:", x, y, z)
            cal_x = map_range(x, mag_min[0], mag_max[0], -1, 1)
            cal_y = map_range(y, mag_min[1], mag_max[1], -1, 1)
            cal_z = map_range(z, mag_min[2], mag_max[2], -1, 1)
            print("Calibrated:  ", cal_x, cal_y, cal_z)
            print("MAG_MIN =", mag_min)
            print("MAG_MAX =", mag_max)

            compass_heading = (math.atan2(cal_y, cal_x) * 180) / math.pi
            if compass_heading < 0:
                compass_heading += 360
            print("Heading: ", compass_heading)
            print("----")

            lastDisplayTime = time.monotonic();

calibrate_compass(compass)
