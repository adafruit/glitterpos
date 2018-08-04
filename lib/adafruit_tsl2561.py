# The MIT License (MIT)
#
# Copyright (c) 2017 Carter Nelson for Adafruit Industries
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
`adafruit_tsl2561`
====================================================

CircuitPython driver for TSL2561 Light Sensor.

* Author(s): Carter Nelson
"""
from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

__version__ = "3.0.1"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_TSL2561.git"

# pylint: disable=bad-whitespace
_DEFAULT_ADDRESS     = const(0x39)
_COMMAND_BIT         = const(0x80)
_WORD_BIT            = const(0x20)

_CONTROL_POWERON     = const(0x03)
_CONTROL_POWEROFF    = const(0x00)

_REGISTER_CONTROL    = const(0x00)
_REGISTER_TIMING     = const(0x01)
_REGISTER_CHAN0_LOW  = const(0x0C)
_REGISTER_CHAN1_LOW  = const(0x0E)
_REGISTER_ID         = const(0x0A)

_GAIN_SCALE = (16, 1)
_TIME_SCALE = (1 / 0.034, 1 / 0.252, 1)
_CLIP_THRESHOLD = (4900, 37000, 65000)
# pylint: enable=bad-whitespace

class TSL2561():
    """Class which provides interface to TSL2561 light sensor."""

    def __init__(self, i2c, address=_DEFAULT_ADDRESS):
        self.buf = bytearray(3)
        self.i2c_device = I2CDevice(i2c, address)
        partno, revno = self.chip_id
        # data sheet says TSL2561 = 0001, reality says 0101
        if not partno == 5:
            raise RuntimeError('Failed to find TSL2561! Part 0x%x Rev 0x%x' % (partno, revno))
        self.enabled = True

    @property
    def chip_id(self):
        """A tuple containing the part number and the revision number."""
        chip_id = self._read_register(_REGISTER_ID)
        partno = (chip_id >> 4) & 0x0f
        revno = chip_id & 0x0f
        return (partno, revno)

    @property
    def enabled(self):
        """The state of the sensor."""
        return (self._read_register(_REGISTER_CONTROL) & 0x03) != 0

    @enabled.setter
    def enabled(self, enable):
        """Enable or disable the sensor."""
        if enable:
            self._enable()
        else:
            self._disable()

    @property
    def lux(self):
        """The computed lux value."""
        return self._compute_lux()

    @property
    def broadband(self):
        """The broadband channel value."""
        return self._read_broadband()

    @property
    def infrared(self):
        """The infrared channel value."""
        return self._read_infrared()

    @property
    def luminosity(self):
        """The overall luminosity as a tuple containing the broadband
        channel and the infrared channel value."""
        return (self.broadband, self.infrared)

    @property
    def gain(self):
        """The gain. 0:1x, 1:16x."""
        return self._read_register(_REGISTER_TIMING) >> 4 & 0x01

    @gain.setter
    def gain(self, value):
        """Set the gain. 0:1x, 1:16x."""
        value &= 0x01
        value <<= 4
        current = self._read_register(_REGISTER_TIMING)
        self.buf[0] = _COMMAND_BIT | _REGISTER_TIMING
        self.buf[1] = (current & 0xef) | value
        with self.i2c_device as i2c:
            i2c.write(self.buf, end=2)

    @property
    def integration_time(self):
        """The integration time. 0:13.7ms, 1:101ms, 2:402ms, or 3:manual"""
        current = self._read_register(_REGISTER_TIMING)
        return current & 0x03

    @integration_time.setter
    def integration_time(self, value):
        """Set the integration time. 0:13.7ms, 1:101ms, 2:402ms, or 3:manual."""
        value &= 0x03
        current = self._read_register(_REGISTER_TIMING)
        self.buf[0] = _COMMAND_BIT | _REGISTER_TIMING
        self.buf[1] = (current & 0xfc) | value
        with self.i2c_device as i2c:
            i2c.write(self.buf, end=2)

    def _compute_lux(self):
        """Based on datasheet for FN package."""
        ch0, ch1 = self.luminosity
        if ch0 == 0:
            return None
        if ch0 > _CLIP_THRESHOLD[self.integration_time]:
            return None
        if ch1 > _CLIP_THRESHOLD[self.integration_time]:
            return None
        ratio = ch1 / ch0
        if ratio >= 0 and ratio <= 0.50:
            lux = 0.0304 * ch0 - 0.062 * ch0 * ratio**1.4
        elif ratio <= 0.61:
            lux = 0.0224 * ch0 - 0.031 * ch1
        elif ratio <= 0.80:
            lux = 0.0128 * ch0 - 0.0153 * ch1
        elif ratio <= 1.30:
            lux = 0.00146 * ch0 - 0.00112 * ch1
        else:
            lux = 0
        # Pretty sure the floating point math formula on pg. 23 of datasheet
        # is based on 16x gain and 402ms integration time. Need to scale
        # result for other settings.
        # Scale for gain.
        lux *= _GAIN_SCALE[self.gain]
        # Scale for integration time.
        lux *= _TIME_SCALE[self.integration_time]
        return lux

    def _enable(self):
        self._write_control_register(_CONTROL_POWERON)

    def _disable(self):
        self._write_control_register(_CONTROL_POWEROFF)

    def _read_register(self, reg, count=1):
        self.buf[0] = _COMMAND_BIT | reg
        if count == 2:
            self.buf[0] |= _WORD_BIT
        with self.i2c_device as i2c:
            i2c.write(self.buf, end=1, stop=False)
            i2c.readinto(self.buf, start=1)
        if count == 1:
            return self.buf[1]
        elif count == 2:
            return self.buf[1], self.buf[2]
        return None

    def _write_control_register(self, reg):
        self.buf[0] = _COMMAND_BIT | _REGISTER_CONTROL
        self.buf[1] = reg
        with self.i2c_device as i2c:
            i2c.write(self.buf, end=2)

    def _read_broadband(self):
        low, high = self._read_register(_REGISTER_CHAN0_LOW, 2)
        return high << 8 | low

    def _read_infrared(self):
        low, high = self._read_register(_REGISTER_CHAN1_LOW, 2)
        return high << 8 | low
