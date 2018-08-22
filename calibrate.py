import time
import math
import adafruit_lsm303
import board
import busio
import neopixel

i2c = busio.I2C(board.SCL, board.SDA)
compass = adafruit_lsm303.LSM303(i2c)

def calibrate_compass(compass):
    AccelMinX = 0
    AccelMaxX = 0
    AccelMinY = 0
    AccelMaxY = 0
    AccelMinZ = 0
    AccelMaxZ = 0

    MagMinX = 0
    MagMaxX = 0
    MagMinY = 0
    MagMaxY = 0
    MagMinZ = 0
    MagMaxZ = 0

    print("LSM303 Calibration")
    lastDisplayTime = time.monotonic()
    while True:

        # Get a new sensor event */
        # sensors_event_t accelEvent;
        # sensors_event_t magEvent;

        acc_x, acc_y, acc_z = compass.raw_acceleration
        mag_x, mag_y, mag_z = compass.raw_magnetic

        if (acc_x < AccelMinX): AccelMinX = acc_x
        if (acc_x > AccelMaxX): AccelMaxX = acc_x

        if (acc_y < AccelMinY): AccelMinY = acc_y
        if (acc_y > AccelMaxY): AccelMaxY = acc_y

        if (acc_z < AccelMinZ): AccelMinZ = acc_z
        if (acc_z > AccelMaxZ): AccelMaxZ = acc_z

        if (mag_x < MagMinX): MagMinX = mag_x
        if (mag_x > MagMaxX): MagMaxX = mag_x

        if (mag_y < MagMinY): MagMinY = mag_y
        if (mag_y > MagMaxY): MagMaxY = mag_y

        if (mag_z < MagMinZ): MagMinZ = mag_z
        if (mag_z > MagMaxZ): MagMaxZ = mag_z

        # display once every two seconds
        if (time.monotonic() - lastDisplayTime >= 2):
            print("Accel Minimums: ")
            print(AccelMinX)
            print(AccelMinY)
            print(AccelMinZ)

            print("Accel Maximums: ")
            print(AccelMaxX)
            print(AccelMaxY)
            print(AccelMaxZ)

            print("Mag Minimums: ")
            print(MagMinX)
            print(MagMinY)
            print(MagMinZ)

            print("Mag Maximums: ")
            print(MagMaxX)
            print(MagMaxY)
            print(MagMaxZ)

            lastDisplayTime = time.monotonic();

calibrate_compass(compass)
