"""glitter positioning system"""


import gc
import rtc
import time
import adafruit_gps
import adafruit_rfm9x
import board
import busio
import digitalio
import neopixel
import math

# Colors for status lights, etc.
RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)

MAN_ID = 23
ELECTRICITY_ID = 42

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
    ELECTRICITY_ID: (220, 47, 207)
}

RADIO_FREQ_MHZ = 915.0
CS = digitalio.DigitalInOut(board.D10)
RESET = digitalio.DigitalInOut(board.D11)

class glitterpos:
    """glitter positioning system"""

    def __init__(self):
        """configure sensors, radio, blinkenlights"""

        # Our id and the dict for storing coords of other Electrons:
        self.electron_id = 1
        self.electrons = {
            MAN_ID: (40.786462, -119.206686),
            # ELECTRICITY_ID: (40.795726, -119.213651) # maybe the real location of camp
            ELECTRICITY_ID: (40.178828, -105.106807) # actually roosevelt park in longmont
        }

        # Set the RTC to an obviously bogus time for debugging purposes:
        # time_struct takes: (tm_year, tm_mon, tm_mday, tm_hour, tm_min, tm_sec, tm_wday, tm_yday, tm_isdst)
        rtc.RTC().datetime = time.struct_time((2000, 1, 1, 0, 0, 0, 0, 0, 0))
        print("startup time: " + self.timestamp())
        self.time_set = False
        self.last_send = time.monotonic()

        self.current_lat = 0
        self.current_lon = 0

        # Status light on the board, we'll use to indicate GPS fix, etc.:
        self.statuslight = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.01, auto_write=True)
        self.statuslight.fill(RED)

        # Neopixel ring:
        self.pixels = neopixel.NeoPixel(board.A1, 16, brightness=0.01, auto_write=False)

        # Startup animation:
        for i in range(len(self.pixels)):
            self.pixels[i] = GREEN
            self.pixels.show()
            time.sleep(0.2)
        self.pixels.fill((0,0,0))
        self.pixels[0] = PURPLE
        self.pixels.show()

        # i2c = busio.I2C(board.SCL, board.SDA)
        # self.bno = adafruit_bno055.BNO055(i2c)
        # self.bno.mode = adafruit_bno055.COMPASS_MODE
        # or!  self.compass = adafruit_lsm303.LSM303(i2c)

        self.init_radio()
        time.sleep(1)

        self.init_gps()
        time.sleep(1)

        self.statuslight.fill(YELLOW)

    def init_radio(self):
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        self.rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
        self.rfm9x.tx_power = 15 # Default is 13 dB, but the RFM95 can go up to 23 dB
        self.radio_send('Hello world!')

    def init_gps(self):
        """Some GPS module setup."""
        uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=3000)
        gps = adafruit_gps.GPS(uart)

        # https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf
        # Turn on the basic GGA and RMC info (what you typically want), then
        # set update to once a second:
        gps.send_command('PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        gps.send_command('PMTK220,1000')

        self.gps = gps

    def advance_frame(self):
        """Main program loop."""

        current = time.monotonic()

        self.check_radio(timeout=0.5)

        new_gps_data = self.gps.update()

        if not self.gps.has_fix:
            self.statuslight.fill(RED)
            # Try again if we don't have a fix yet.
            return

        self.display_pixels()

        if (not new_gps_data) and (current - self.last_send < 10):
            return

        # Set the RTC to GPS time (UTC):
        # XXX: think about this a bit - reset it periodically?
        if new_gps_data and not self.time_set:
            # rtc.RTC().datetime = time.struct_time((2018, 3, 17, 21, 1, 47, 0, 0, 0))
            rtc.RTC().datetime = self.gps.timestamp_utc
            self.time_set = True

        if (self.gps.latitude != self.current_lat) or (self.gps.longitude != self.current_lon):
            self.current_lat = self.gps.latitude
            self.current_lon = self.gps.longitude
        else:
            return

        self.statuslight.fill(BLUE)
        print(':: ' + str(current))  # Print a separator line.
        print(self.timestamp())
        send_packet = "{}\t{}\t{}\t{}".format(
            self.gps.latitude,
            self.gps.longitude,
            self.gps.speed_knots,
            self.gps.track_angle_deg
        )

        print('   quality: {}'.format(self.gps.fix_quality))
        print(self.timestamp())
        self.radio_send(send_packet)
        print('   ' + str(gc.mem_free()) + " bytes free")

    def radio_send(self, msg):
        """send a packet over radio with id prefix and checksum"""
        # XXX: implement checksum
        send_packet = "e\t" + str(self.electron_id) + "\t" + msg
        print("   sending: " + send_packet)

        # Blocking, max of 252 bytes:
        self.rfm9x.send(send_packet)
        self.last_send = time.monotonic()

    def check_radio(self, timeout=0.5):
        """check radio for new packets, handle incoming data"""

        packet = self.rfm9x.receive(timeout)

        # If no packet was received during the timeout then None is returned:
        if packet is None:
            return

        rssi = self.rfm9x.rssi
        print(self.timestamp())
        print('   received signal strength: {0} dB'.format(rssi))
        print('   received (raw bytes): {0}'.format(packet))

        # if packet.startswith(b'e'):
        #    print("got prefix")

        # Ok, I'm going to bed, but here is where I want to define:
        #   - a delimiter, so it's broken into fields
        #   - first field is prefix, second is sender id, third is message type,
        #     remainder are message fields.
        # this is what 252 bytes looks like:
        # xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

        # that's actually a ton of room.  can definitely fit an id and coordinates in there.  what else would
        # be worth having?  gps heading and speed, i suppose.

        # packet_text = str(packet, 'ascii')
        # print('Received (ASCII): {0}'.format(packet_text))

    def display_pixels(self):
        self.pixels.fill((0, 0, 0))

        for electron in self.electrons:
            angle_to_electron = self.compass_bearing((self.current_lat, self.current_lon), self.electrons[electron])
            # print('angle to ' + str(electron) + ': ' + str(angle_to_electron))

            # Subtract from 16 since the neopixel ring runs counterclockwise:
            pixel = 16 - int(round((angle_to_electron / 360) * 16))
            self.pixels[pixel] = COLOR_LOOKUP[electron]

        self.pixels.show()

    # https://gist.githubusercontent.com/jeromer/2005586/raw/5456a9386acce189ac6cc416c42e9c4b560a633b/compassbearing.py
    def compass_bearing(self, pointA, pointB):
        """
        Calculates the bearing between two points.

        The formulae used is the following:
            θ = atan2(sin(Δlong).cos(lat2),
                      cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong))

        :Parameters:
          - `pointA: The tuple representing the latitude/longitude for the
            first point. Latitude and longitude must be in decimal degrees
          - `pointB: The tuple representing the latitude/longitude for the
            second point. Latitude and longitude must be in decimal degrees

        :Returns:
          The bearing in degrees

        :Returns Type:
          float
        """
        if (type(pointA) != tuple) or (type(pointB) != tuple):
            raise TypeError("Only tuples are supported as arguments")

        lat1 = math.radians(pointA[0])
        lat2 = math.radians(pointB[0])

        diffLong = math.radians(pointB[1] - pointA[1])

        x = math.sin(diffLong) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
                * math.cos(lat2) * math.cos(diffLong))

        initial_bearing = math.atan2(x, y)

        # Now we have the initial bearing but math.atan2 return values
        # from -180° to + 180° which is not what we want for a compass bearing
        # The solution is to normalize the initial bearing as shown below
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing

    def timestamp(self):
        """print a human-readable timestamp"""
        timestamp = time.localtime()
        return '{}/{}/{} {:02}:{:02}:{:02}'.format(
            timestamp.tm_year,
            timestamp.tm_mon,
            timestamp.tm_mday,
            timestamp.tm_hour,
            timestamp.tm_min,
            timestamp.tm_sec
        )
