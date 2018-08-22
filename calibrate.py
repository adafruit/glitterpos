# Will determine minimum and maximum magnet values

import time
import board
import busio
import adafruit_lsm9ds1

# I2C connection:

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

        # display once every two seconds
        if (time.monotonic() - lastDisplayTime >= 2):
            print("Mag Min: ", mag_min, "& Max:", mag_max)
            lastDisplayTime = time.monotonic();

calibrate_compass(compass)
