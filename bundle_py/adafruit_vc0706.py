# The MIT License (MIT)
#
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_vc0706`
====================================================

VC0706 serial TTL camera module.  Allows basic image capture and download of
image data from the camera over a serial connection.  See examples for demo
of saving image to a SD card (must be wired up separately).

* Author(s): Tony DiCola

Implementation Notes
--------------------

**Hardware:**

* Adafruit `TTL Serial JPEG Camera with NTSC Video
  <https://www.adafruit.com/product/397>`_ (Product ID: 397)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the ESP8622 and M0-based boards:
  https://github.com/adafruit/circuitpython/releases
"""
import busio
from micropython import const

__version__ = "3.0.2"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_VC0706.git"

# pylint: disable=bad-whitespace
_SERIAL               = const(0x00)
_RESET                = const(0x26)
_GEN_VERSION          = const(0x11)
_SET_PORT             = const(0x24)
_READ_FBUF            = const(0x32)
_GET_FBUF_LEN         = const(0x34)
_FBUF_CTRL            = const(0x36)
_DOWNSIZE_CTRL        = const(0x54)
_DOWNSIZE_STATUS      = const(0x55)
_READ_DATA            = const(0x30)
_WRITE_DATA           = const(0x31)
_COMM_MOTION_CTRL     = const(0x37)
_COMM_MOTION_STATUS   = const(0x38)
_COMM_MOTION_DETECTED = const(0x39)
_MOTION_CTRL          = const(0x42)
_MOTION_STATUS        = const(0x43)
_TVOUT_CTRL           = const(0x44)
_OSD_ADD_CHAR         = const(0x45)

_STOPCURRENTFRAME     = const(0x0)
_STOPNEXTFRAME        = const(0x1)
_RESUMEFRAME          = const(0x3)
_STEPFRAME            = const(0x2)

# pylint doesn't like the lowercase x but it makes it more readable.
# pylint: disable=invalid-name
IMAGE_SIZE_640x480              = const(0x00)
IMAGE_SIZE_320x240              = const(0x11)
IMAGE_SIZE_160x120              = const(0x22)
# pylint: enable=invalid-name

_MOTIONCONTROL        = const(0x0)
_UARTMOTION           = const(0x01)
_ACTIVATEMOTION       = const(0x01)

__SET_ZOOM             = const(0x52)
__GET_ZOOM             = const(0x53)

_CAMERA_DELAY               = const(10)
# pylint: enable=bad-whitespace


class VC0706:
    """Driver for VC0706 serial TTL camera module.

       :param ~microcontroller.Pin rx: Receive pin
       :param ~microcontroller.Pin tx: Transmit pin
       :param int baudrate: Serial connection speed
       :param int timeout: Read timeout in seconds
       :param int buffer_size: Receive buffer size
    """
    def __init__(self, rx, tx, *, baudrate=38400, timeout=250, buffer_size=100):
        self._uart = busio.UART(tx, rx, baudrate=baudrate, timeout=timeout)
        self._buffer = bytearray(buffer_size)
        self._frame_ptr = 0
        self._command_header = bytearray(3)
        if not self._run_command(_RESET, bytes([0x00]), 5):
            raise RuntimeError('Failed to get response from VC0706, check wiring!')

    @property
    def version(self):
        """Return camera version byte string."""
        # Clear buffer to ensure the end of a string can be found.
        self._send_command(_GEN_VERSION, bytes([0x01]))
        readlen = self._read_response(self._buffer, len(self._buffer))
        return str(self._buffer[:readlen], 'ascii')

    @property
    def image_size(self):
        """Get the current image size, will return a value of IMAGE_SIZE_640x480,
           IMAGE_SIZE_320x240, or IMAGE_SIZE_160x120.
        """
        if not self._run_command(_READ_DATA,
                                 bytes([0x4, 0x4, 0x1, 0x00, 0x19]),
                                 6):
            raise RuntimeError('Failed to read image size!')
        return self._buffer[5]

    @image_size.setter
    def image_size(self, size):
        """Set the image size to a value of IMAGE_SIZE_640x480, IMAGE_SIZE_320x240, or
           IMAGE_SIZE_160x120.
        """
        if size not in (IMAGE_SIZE_640x480, IMAGE_SIZE_320x240, IMAGE_SIZE_160x120):
            raise ValueError("Size must be one of IMAGE_SIZE_640x480, IMAGE_SIZE_320x240, or "
                             "IMAGE_SIZE_160x120!")
        return self._run_command(_WRITE_DATA, bytes([0x05, 0x04, 0x01, 0x00, 0x19, size & 0xFF]), 5)

    @property
    def frame_length(self):
        """Return the length in bytes of the currently capture frame/picture.
        """
        if not self._run_command(_GET_FBUF_LEN, bytes([0x01, 0x00]), 9):
            return 0
        frame_length = self._buffer[5]
        frame_length <<= 8
        frame_length |= self._buffer[6]
        frame_length <<= 8
        frame_length |= self._buffer[7]
        frame_length <<= 8
        frame_length |= self._buffer[8]
        return frame_length

    def take_picture(self):
        """Tell the camera to take a picture.  Returns True if successful.
        """
        self._frame_ptr = 0
        return self._run_command(_FBUF_CTRL, bytes([0x1, _STOPCURRENTFRAME]), 5)

    def read_picture_into(self, buf):
        """Read the next bytes of frame/picture data into the provided buffer.
        Returns the number of bytes written to the buffer (might be less than
        the size of the buffer).  Buffer MUST be a multiple of 4 and 100 or
        less.  Suggested buffer size is 32.
        """
        n = len(buf)
        if n > 256 or n > (len(self._buffer)-5):
            raise ValueError('Buffer is too large!')
        if n % 4 != 0:
            raise ValueError('Buffer must be a multiple of 4! Try 32.')
        args = bytes([0x0C, 0x0, 0x0A, 0, 0, (self._frame_ptr >> 8) & 0xFF,
                      self._frame_ptr & 0xFF, 0, 0, 0, n & 0xFF,
                      (_CAMERA_DELAY >> 8) & 0xFF, _CAMERA_DELAY & 0xFF])
        if not self._run_command(_READ_FBUF, args, 5):
            return 0
        if self._read_response(self._buffer, n+5) == 0:
            return 0
        self._frame_ptr += n
        for i in range(n):
            buf[i] = self._buffer[i]
        return n

    def _run_command(self, cmd, args, resplen, flush=True):
        if flush:
            self._read_response(self._buffer, len(self._buffer))
        self._send_command(cmd, args)
        if self._read_response(self._buffer, resplen) != resplen:
            return False
        if not self._verify_response(cmd):
            return False
        return True

    def _read_response(self, result, numbytes):
        return self._uart.readinto(result, numbytes)

    def _verify_response(self, cmd):
        return (self._buffer[0] == 0x76 and self._buffer[1] == _SERIAL and
                self._buffer[2] == cmd & 0xFF and self._buffer[3] == 0x00)

    def _send_command(self, cmd, args=None):
        self._command_header[0] = 0x56
        self._command_header[1] = _SERIAL
        self._command_header[2] = cmd & 0xFF
        self._uart.write(self._command_header)
        if args:
            self._uart.write(args)
