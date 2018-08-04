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
`Adafruit_MAX9744`
====================================================

CircuitPython module for the MAX9744 20W class D amplifier.  See
examples/simpletest.py for a demo of the usage.

* Author(s): Tony DiCola
"""
from micropython import const

__version__ = "1.0.2"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MAX9744.git"


#pylint: disable=bad-whitespace
# Internal constants:
_MAX9744_DEFAULT_ADDRESS     = const(0b01001011)
_MAX9744_COMMAND_VOLUME      = const(0b00000000)
_MAX9744_COMMAND_FILTERLESS  = const(0b01000000)
_MAX9744_COMMAND_CLASSIC_PWM = const(0b01000001)
_MAX9744_COMMAND_VOLUME_UP   = const(0b11000100)
_MAX9744_COMMAND_VOLUME_DOWN = const(0b11000101)
#pylint: enable=bad-whitespace


class MAX9744:
    """MAX9744 20 watt class D amplifier.  Construct this by passing the
    following parameters:
    - i2c: The I2C bus for the device.

    Optionally specify:
    - address: The address of the device if it has been overridden from the
               default with the AD1, AD2 pins.
    """

    # Global buffer for writing data.  This saves memory use and prevents
    # heap fragmentation.  However this is not thread-safe or re-entrant by
    # design!
    _BUFFER = bytearray(1)

    def __init__(self, i2c, *, address=_MAX9744_DEFAULT_ADDRESS):
        # This device doesn't use registers and instead just accepts a single
        # command string over I2C.  As a result we don't use bus device or
        # other abstractions and just talk raw I2C protocol.
        self._i2c = i2c
        self._address = address

    def _write(self, val):
        # Perform a write to update the amplifier state.
        try:
            # Make sure bus is locked before write.
            while not self._i2c.try_lock():
                pass
            # Build bytes to send to device with updated value.
            self._BUFFER[0] = val & 0xFF
            self._i2c.writeto(self._address, self._BUFFER)
        finally:
            # Ensure bus is always unlocked.
            self._i2c.unlock()

    def _set_volume(self, volume):
        # Set the volume to the specified level (0-63).
        assert 0 <= volume <= 63
        self._write(_MAX9744_COMMAND_VOLUME | (volume & 0x3F))

    #pylint: disable=line-too-long
    volume = property(None, _set_volume, "Set the volume of the amplifier.  Specify a value from 0-63 where 0 is muted/off and 63 is maximum volume.")
    #pylint: enable=line-too-long

    def volume_up(self):
        """Increase the volume by one level."""
        self._write(_MAX9744_COMMAND_VOLUME_UP)

    def volume_down(self):
        """Decrease the volume by one level."""
        self._write(_MAX9744_COMMAND_VOLUME_DOWN)
