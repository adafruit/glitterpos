"""glitter positioning system"""

import time
import gc
import math
import adafruit_lsm9ds1
import adafruit_gps
import adafruit_rfm9x
import board
import busio
import digitalio
import neopixel
import rtc
from glitterpos_util import timestamp, compass_bearing, bearing_to_pixel, map_range

# glitterpos_cfg.py should be unique to each box, and formatted as follows:
#
# MY_ID = 0 # must be a unique integer
# MAG_MIN = (-0.25046, -0.23506, -0.322)
# MAG_MAX = (0.68278, 0.70882, 0.59654)
# DECLINATION_RAD = 235.27 / 1000.0 # Black Rock City in radians
#
# From the CircuitPython REPL, use `import calibrate` to find values for
# MAG_MIN and MAG_MAX.
from glitterpos_cfg import MY_ID, MAG_MIN, MAG_MAX, DECLINATION_RAD

# Colors for status lights, etc.
RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)

MAN_ID = 23

COLOR_LOOKUP = {
    0: GREEN,
    1: BLUE,
    2: PURPLE,
    3: YELLOW,
    4: CYAN,
    5: (100, 0, 255),
    6: (0, 100, 200),
    7: (100, 50, 100),
    MAN_ID: RED,
}

RADIO_FREQ_MHZ = 915.0
CS = digitalio.DigitalInOut(board.D10)
RESET = digitalio.DigitalInOut(board.D11)

class GlitterPOS:
    """glitter positioning system"""

    def __init__(self):
        """configure sensors, radio, blinkenlights"""

        # Our id and the dict for storing coords of other glitterpos_boxes:
        self.glitterpos_id = MY_ID
        self.glitterpos_boxes = {
            MAN_ID: (40.786462, -119.206686),
        }

        # Set the RTC to an obviously bogus time for debugging purposes:
        # time_struct takes: (tm_year, tm_mon, tm_mday, tm_hour, tm_min, tm_sec, tm_wday, tm_yday, tm_isdst)
        rtc.RTC().datetime = time.struct_time((2000, 1, 1, 0, 0, 0, 0, 0, 0))
        print("startup time: " + timestamp())
        self.time_set = False
        self.last_send = time.monotonic()

        # A tuple for our lat/long:
        self.coords = (0, 0)
        self.heading = 0.0

        # Status light on the board, we'll use to indicate GPS fix, etc.:
        self.statuslight = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.005, auto_write=True)
        self.statuslight.fill(RED)

        # Neopixel ring:
        self.pixels = neopixel.NeoPixel(board.A1, 16, brightness=0.01, auto_write=False)
        self.startup_animation()
        time.sleep(2)

        self.init_radio()
        self.init_gps()
        self.init_compass()

        self.statuslight.fill(YELLOW)

    def startup_animation(self):
        self.pixels[bearing_to_pixel(0)] = PURPLE
        self.pixels.show()
        time.sleep(.5)
        self.pixels[bearing_to_pixel(90)] = GREEN
        self.pixels.show()
        time.sleep(.5)
        self.pixels[bearing_to_pixel(180)] = YELLOW
        self.pixels.show()
        time.sleep(.5)
        self.pixels[bearing_to_pixel(270)] = BLUE
        self.pixels.show()

    def init_radio(self):
        """Set up RFM95."""
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        self.rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
        self.rfm9x.tx_power = 18 # Default is 13 dB, but the RFM95 can go up to 23 dB
        self.radio_tx('d', 'hello world')
        time.sleep(1)

    def init_gps(self):
        """Some GPS module setup."""
        uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=3000)
        gps = adafruit_gps.GPS(uart)
        time.sleep(1)

        # https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf
        # Turn on the basic GGA and RMC info (what you typically want), then
        # set update to once a second:
        gps.send_command('PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        gps.send_command('PMTK220,1000')

        self.gps = gps

    def init_compass(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.compass = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
        time.sleep(1)

    def advance_frame(self):
        """Essentially our main program loop."""

        current = time.monotonic()
        self.radio_rx(timeout=0.5)
        new_gps_data = self.gps.update()
        self.update_heading()
        self.display_pixels()

        if not self.gps.has_fix:
            # Try again if we don't have a fix yet.
            self.statuslight.fill(RED)
            return

        # We want to send coordinates out either on new GPS data or roughly every 15 seconds.
        if (not new_gps_data) and (current - self.last_send < 15):
            return

        # Set the RTC to GPS time (UTC):
        if new_gps_data and not self.time_set:
            rtc.RTC().datetime = self.gps.timestamp_utc
            self.time_set = True

        gps_coords = (self.gps.latitude, self.gps.longitude)
        if gps_coords == self.coords:
            return

        self.coords = (self.gps.latitude, self.gps.longitude)

        self.statuslight.fill(BLUE)
        print(':: ' + str(current))  # Print a separator line.
        print(timestamp())
        send_packet = '{}:{}:{}:{}'.format(
            self.gps.latitude,
            self.gps.longitude,
            self.gps.speed_knots,
            self.heading
        )

        print('   quality: {}'.format(self.gps.fix_quality))
        print('   ' + str(gc.mem_free()) + " bytes free")

        # Send a location packet:
        self.radio_tx('l', send_packet)

    def update_heading(self):
        mag_x, mag_y, mag_z = self.compass.magnetometer
        # print('Magnetometer: ({0:10.3f}, {1:10.3f}, {2:10.3f})'.format(mag_x, mag_y, mag_z))
        mag_x = map_range(mag_x, MAG_MIN[0], MAG_MAX[0], -1, 1)
        mag_y = map_range(mag_y, MAG_MIN[1], MAG_MAX[1], -1, 1)
        mag_z = map_range(mag_z, MAG_MIN[2], MAG_MAX[2], -1, 1)

        heading_mag = (math.atan2(mag_y, mag_x) * 180) / math.pi
        if heading_mag < 0:
            heading_mag = 360 + heading_mag

        # Account for declination (given in radians above):
        heading = heading_mag + (DECLINATION_RAD * 180 / math.pi)
        if heading > 360:
            heading = heading - 360

        print('heading: {}'.format(heading))
        self.heading = heading

    def radio_tx(self, msg_type, msg):
        """send a packet over radio with id prefix and checksum"""
        packet = 'e:' + msg_type + ':' + str(self.glitterpos_id) + ':' + msg
        print('   sending: ' + packet)

        # Blocking, max of 252 bytes:
        self.rfm9x.send(packet)
        self.last_send = time.monotonic()

    def radio_rx(self, timeout=0.5):
        """check radio for new packets, handle incoming data"""

        packet = self.rfm9x.receive(timeout)

        # If no packet was received during the timeout then None is returned:
        if packet is None:
            return

        packet = bytes(packet)
        print(timestamp())
        print('   received signal strength: {0} dB'.format(self.rfm9x.rssi))
        print('   received (raw bytes): {0}'.format(packet))
        pieces = packet.split(b':')

        if pieces[0] != b'e':
            print('   bogus packet, bailing out')
            return

        msg_type = pieces[1].format()
        sender_id = int(pieces[2].format())

        # A location message:
        if msg_type == 'l':
            sender_lat = float(pieces[3].format())
            sender_lon = float(pieces[4].format())
            self.glitterpos_boxes[sender_id] = (sender_lat, sender_lon)

        # packet_text = str(packet, 'ascii')
        # print('Received (ASCII): {0}'.format(packet_text))

    def display_pixels(self):
        """Display current state on the NeoPixel ring."""
        self.pixels.fill((0, 0, 0))

        if not self.gps.has_fix:
            return

        for box in self.glitterpos_boxes:
            bearing_to_box = compass_bearing(self.coords, self.glitterpos_boxes[box])

            # Treat current compass heading as our origin point for display purposes:
            display_bearing = bearing_to_box - self.heading
            if display_bearing < 0:
                display_bearing = display_bearing + 360

            pixel = bearing_to_pixel(display_bearing)
            # print('display pixel: {}'.format(pixel))

            color = (15, 15, 15)
            if box in COLOR_LOOKUP:
                color = COLOR_LOOKUP[box]
            self.pixels[pixel] = color

        self.pixels.show()
