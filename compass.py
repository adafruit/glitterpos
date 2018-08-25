import time
import math
import adafruit_lsm9ds1
import board
import busio
import neopixel

PURPLE = (180, 0, 255)

i2c = busio.I2C(board.SCL, board.SDA)
compass = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
pixels = neopixel.NeoPixel(board.A1, 16, brightness=0.01, auto_write=False)

# (x, y, z) tuples:
MAG_MIN = (-0.25046, -0.23506, -0.322)
MAG_MAX = (0.68278, 0.70882, 0.59654)

def bearing_to_pixel(bearing, count=16):
    pixel = count - int(round((bearing / 360) * count))
    if (pixel == 16):
        pixel = 0
    return pixel

def map_range(x, in_min, in_max, out_min, out_max):
    """
    Maps a number from one range to another.
    :return: Returns value mapped to new range
    :rtype: float
    """
    mapped = (x-in_min) * (out_max - out_min) / (in_max-in_min) + out_min
    if out_min <= out_max:
        return max(min(mapped, out_max), out_min)

    return min(max(mapped, out_max), out_min)

while True:
    mag_x, mag_y, mag_z = compass.magnetometer

    mag_x = map_range(mag_x, MAG_MIN[0], MAG_MAX[0], -1, 1)
    mag_y = map_range(mag_y, MAG_MIN[1], MAG_MAX[1], -1, 1)
    mag_z = map_range(mag_z, MAG_MIN[2], MAG_MAX[2], -1, 1)

    compass_heading = (math.atan2(mag_y, mag_x) * 180) / math.pi;
    if compass_heading < 0:
        compass_heading = 360 + compass_heading;

    pixel = bearing_to_pixel(compass_heading)

    print('Magnetometer: ({0:10.3f}, {1:10.3f}, {2:10.3f})'.format(mag_x, mag_y, mag_z))
    print(compass_heading)
    print('Lighting pixel: {}'.format(pixel))

    pixels.fill((0,0,0))
    pixels[pixel] = PURPLE
    pixels.show()

    time.sleep(0.2)
