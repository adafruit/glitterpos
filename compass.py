import time
import math
import adafruit_lsm303
import board
import busio
import neopixel

PURPLE = (180, 0, 255)

i2c = busio.I2C(board.SCL, board.SDA)
compass = adafruit_lsm303.LSM303(i2c)
pixels = neopixel.NeoPixel(board.A1, 16, brightness=0.01, auto_write=False)

def bearing_to_pixel(bearing, count=16):
    pixel = count - int(round((bearing / 360) * count))
    if (pixel == 16):
        pixel = 0
    return pixel

while True:

# float heading = 180 * atan2(magRaw[1],magRaw[0])/M_PI;
# if(heading < 0)
#     heading += 360;

    raw_mag_x, raw_mag_y, raw_mag_z = compass.raw_magnetic
    compass_heading = (math.atan2(raw_mag_y, raw_mag_x) * 180) / math.pi;
    if compass_heading < 0:
        compass_heading = 360 + compass_heading;

    pixel = bearing_to_pixel(compass_heading)

    print('Magnetometer: ({0:10.3f}, {1:10.3f}, {2:10.3f})'.format(raw_mag_x, raw_mag_y, raw_mag_z))
    print(compass_heading)
    print('Lighting pixel: {}'.format(pixel))

    pixels.fill((0,0,0))
    pixels[pixel] = PURPLE
    pixels.show()

    time.sleep(0.2)
