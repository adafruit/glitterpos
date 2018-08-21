"""glitter positioning system"""

import time
import gc
import adafruit_gps
import adafruit_rfm9x
import board
import busio
import crc16pure
import digitalio
import neopixel
import rtc
from glitterpos_util import timestamp, compass_bearing
from glitterpos_id import MY_ID

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

class GlitterPOS:
    """glitter positioning system"""

    def __init__(self):
        """configure sensors, radio, blinkenlights"""

        # Our id and the dict for storing coords of other Electrons:
        self.electron_id = MY_ID
        self.electrons = {
            MAN_ID: (40.786462, -119.206686),
            # ELECTRICITY_ID: (40.795726, -119.213651) # maybe the real location of camp
            ELECTRICITY_ID: (40.178828, -105.106807) # actually roosevelt park in longmont
        }

        # Set the RTC to an obviously bogus time for debugging purposes:
        # time_struct takes: (tm_year, tm_mon, tm_mday, tm_hour, tm_min, tm_sec, tm_wday, tm_yday, tm_isdst)
        rtc.RTC().datetime = time.struct_time((2000, 1, 1, 0, 0, 0, 0, 0, 0))
        print("startup time: " + timestamp())
        self.time_set = False
        self.last_send = time.monotonic()

        # A tuple for our lat/long:
        self.coords = (0, 0)

        # Status light on the board, we'll use to indicate GPS fix, etc.:
        self.statuslight = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.005, auto_write=True)
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

        self.init_radio()
        time.sleep(1)

        self.init_gps()
        time.sleep(1)

        self.statuslight.fill(YELLOW)

    def init_radio(self):
        """Set up RFM95."""
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        self.rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
        self.rfm9x.tx_power = 18 # Default is 13 dB, but the RFM95 can go up to 23 dB
        self.radio_tx('d', 'hello world')

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
        """Essentially our main program loop."""

        current = time.monotonic()

        self.radio_rx(timeout=0.5)

        new_gps_data = self.gps.update()

        if not self.gps.has_fix:
            # Try again if we don't have a fix yet.
            self.statuslight.fill(RED)
            return

        self.display_pixels()

        # We want to send coordinates out either on new GPS data or roughly every 15 seconds.
        if (not new_gps_data) and (current - self.last_send < 15):
            return

        # Set the RTC to GPS time (UTC):
        # XXX: think about this a bit - reset it periodically?
        if new_gps_data and not self.time_set:
            rtc.RTC().datetime = self.gps.timestamp_utc
            self.time_set = True

        gps_coords = (self.gps.latitude, self.gps.longitude)
        if (gps_coords == self.coords):
            return

        self.coords = (self.gps.latitude, self.gps.longitude)

        self.statuslight.fill(BLUE)
        print(':: ' + str(current))  # Print a separator line.
        print(timestamp())
        send_packet = '{}:{}:{}:{}'.format(
            self.gps.latitude,
            self.gps.longitude,
            self.gps.speed_knots,
            self.gps.track_angle_deg
        )

        print('   quality: {}'.format(self.gps.fix_quality))
        print('   ' + str(gc.mem_free()) + " bytes free")

        # Send a location packet:
        self.radio_tx('l', send_packet)

    def radio_tx(self, msg_type, msg):
        """send a packet over radio with id prefix and checksum"""
        # XXX: implement checksum?
        packet = 'e:' + msg_type + ':' + str(self.electron_id) + ':' + msg
        packet_with_crc = packet + ':' + str(crc16pure.crc16xmodem(bytes(packet, 'ascii')))
        print('   sending: ' + packet_with_crc)

        # Blocking, max of 252 bytes:
        self.rfm9x.send(packet_with_crc)
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
            self.electrons[sender_id] = (sender_lat, sender_lon)

        # packet_text = str(packet, 'ascii')
        # print('Received (ASCII): {0}'.format(packet_text))

    def display_pixels(self):
        self.pixels.fill((0, 0, 0))

        for electron in self.electrons:
            angle_to_electron = compass_bearing(self.coords, self.electrons[electron])
            # print('angle to ' + str(electron) + ': ' + str(angle_to_electron))

            # Subtract from 16 since the neopixel ring runs counterclockwise:
            pixel = 16 - int(round((angle_to_electron / 360) * 16))
            color = (15, 15, 15)
            if electron in COLOR_LOOKUP:
                color = COLOR_LOOKUP[electron]
            self.pixels[pixel] = color

        self.pixels.show()
