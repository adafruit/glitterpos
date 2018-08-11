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
`adafruit_tlc5947`
====================================================

CircuitPython module for the TLC5947 12-bit 24 channel LED PWM driver.  See
examples/simpletest.py for a demo of the usage.

* Author(s): Tony DiCola

Implementation Notes
--------------------

**Hardware:**

* Adafruit `24-Channel 12-bit PWM LED Driver - SPI Interface - TLC5947
  <https://www.adafruit.com/product/1429>`_ (Product ID: 1429)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the ESP8622 and M0-based boards:
  https://github.com/adafruit/circuitpython/releases
"""
__version__ = "1.0.1"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_TLC5947.git"


# Globally disable protected access.  Ppylint can't figure out the
# context for using internal decorate classes below.  In these cases protectected
# access is by design for the internal class.
#pylint: disable=protected-access


class TLC5947:
    """TLC5947 12-bit 24 channel LED PWM driver.  Create an instance of this by
    passing in at least the following parameters:

    :param spi: The SPI bus connected to the chip (only the SCK and MOSI lines are
                used, there is no MISO/input).
    :param latch: A DigitalInOut instance connected to the chip's latch line.

    Optionally you can specify:

    :param auto_write: This is a boolean that defaults to True and will automatically
                       write out all the channel values to the chip as soon as a
                       single one is updated.  If you set to false to disable then
                       you MUST call write after every channel update or when you
                       deem necessary to update the chip state.
    """

    class PWMOut:
        """Internal PWMOut class that mimics the behavior of CircuitPython's
        PWMOut class but is associated with a channel on the TLC5947.  You can
        get and set the instance's duty_cycle property as a 16-bit PWM value
        (note there will be quantization errors as the TLC5947 is a 12-bit PWM
        chip, instead use the TLC5947 class item accessor notation for direct
        12-bit raw PWM channel access).  Note you cannot change the frequency
        as it is fixed by the TLC5947 to ~2.4-5.6 mhz.
        """

        def __init__(self, tlc5947, channel):
            self._tlc5947 = tlc5947
            self._channel = channel

        @property
        def duty_cycle(self):
            """Get and set the 16-bit PWM duty cycle value for this channel.
            """
            raw_value = self._tlc5947._get_gs_value(self._channel)
            # Convert to 16-bit value from 12-bits and return it.
            return (raw_value << 4) & 0xFFFF

        @duty_cycle.setter
        def duty_cycle(self, val):
            assert 0 <= val <= 65535
            # Convert to 12-bit value (quantization error will occur!).
            val = (val >> 4) & 0xFFF
            self._tlc5947._set_gs_value(self._channel, val)

        @property
        def frequency(self):
            """Frequency of the PWM channel, note you cannot change this and
            cannot read its exact value (it varies from 2.4-5.6 mhz, see the
            TLC5947 datasheet).
            """
            return 0

        # pylint bug misidentifies the following as a regular function instead
        # of the associated setter: https://github.com/PyCQA/pylint/issues/870
        # Must disable a few checks to make pylint happy (ugh).
        #pylint: disable=no-self-use,unused-argument
        @frequency.setter
        def frequency(self, val):
            raise RuntimeError('Cannot set TLC5947 PWM frequency!')
        #pylint: enable=no-self-use,unused-argument


    def __init__(self, spi, latch, *, auto_write=True):
        self._spi = spi
        self._latch = latch
        self._latch.switch_to_output(value=False)
        # This device is just a big 36 byte long shift register.  There's no
        # fancy protocol or other commands to send, just write out all 288
        # bits every time the state is updated.
        self._shift_reg = bytearray(36)
        # Save auto_write state (i.e. push out shift register values on
        # any channel value change).
        self.auto_write = auto_write

    def write(self):
        """Write out the current channel PWM values to the chip.  This is only
        necessary to call if you disabled auto_write in the initializer,
        otherwise write is automatically called on any channel update.
        """
        # Write out the current state to the shift register.
        try:
            # Lock the SPI bus and configure it for the shift register.
            while not self._spi.try_lock():
                pass
            self._spi.configure(baudrate=1000000, polarity=0, phase=0, bits=8)
            # First ensure latch is low.
            self._latch.value = False
            # Write out the bits.
            self._spi.write(self._shift_reg, start=0, end=37)
            # Then toggle latch high and low to set the value.
            self._latch.value = True
            self._latch.value = False
        finally:
            # Ensure the SPI bus is unlocked.
            self._spi.unlock()

    def _get_gs_value(self, channel):
        assert 0 <= channel <= 23
        # Invert channel position as the last channel needs to be written first.
        # I.e. is in the first position of the shift registr.
        channel = 23 - channel
        # Calculate exact bit position within the shift register.
        bit_offset = channel * 12
        # Now calculate the byte that this position falls within and any offset
        # from the left inside that byte.
        byte_start = bit_offset // 8
        start_offset = bit_offset % 8
        # Grab the high and low bytes.
        high_byte = self._shift_reg[byte_start]
        low_byte = self._shift_reg[byte_start+1]
        if start_offset == 4:
            # Value starts in the lower 4 bits of the high bit so you can
            # just concat high with low byte and return the 12-bit value.
            return ((high_byte << 8) | low_byte) & 0xFFF
        elif start_offset == 0:
            # Value starts in the entire high byte and spills into upper
            # 4 bits of low byte.  Shift low byte and concat values.
            return ((high_byte << 4) | (low_byte >> 4)) & 0xFFF
        else:
            raise RuntimeError('Unsupported bit offset!')

    def _set_gs_value(self, channel, val):
        assert 0 <= channel <= 23
        assert 0 <= val <= 4095
        # Invert channel position as the last channel needs to be written first.
        # I.e. is in the first position of the shift registr.
        channel = 23 - channel
        # Calculate exact bit position within the shift register.
        bit_offset = channel * 12
        # Now calculate the byte that this position falls within and any offset
        # from the left inside that byte.
        byte_start = bit_offset // 8
        start_offset = bit_offset % 8
        # Grab the high and low bytes.
        high_byte = self._shift_reg[byte_start]
        low_byte = self._shift_reg[byte_start+1]
        if start_offset == 4:
            # Value starts in the lower 4 bits of the high bit.
            high_byte &= 0b11110000
            high_byte |= (val >> 8)
            low_byte = val & 0xFF
        elif start_offset == 0:
            # Value starts in the entire high byte and spills into upper
            # 4 bits of low byte.
            high_byte = (val >> 4) & 0xFF
            low_byte &= 0b00001111
            low_byte |= (val << 4) & 0xFF
        else:
            raise RuntimeError('Unsupported bit offset!')
        self._shift_reg[byte_start] = high_byte
        self._shift_reg[byte_start+1] = low_byte
        # Write the updated shift register values if required.
        if self.auto_write:
            self.write()

    def create_pwm_out(self, channel):
        """Create an instance of a PWMOut-like class that mimics the built-in
        CircuitPython PWMOut class but is associated with the TLC5947 channel
        that is specified.  This PWMOut class has a duty_cycle property which
        you can read and write with a 16-bit value to control the channel.
        Note there will be quantization error as the chip only supports 12-bit
        PWM, if this is problematic use the item accessor approach to update
        the raw 12-bit channel values.
        """
        return self.PWMOut(self, channel)

    # Define index and length properties to set and get each channel's raw
    # 12-bit value (useful for changing channels without quantization error
    # like when using the PWMOut mock class).
    def __len__(self):
        """Retrieve the total number of PWM channels available."""
        return 24  # Always 24 channels on the chip.

    def __getitem__(self, key):
        """Retrieve the 12-bit PWM value for the specified channel (0-23).
        """
        assert 0 <= key <= 23
        return self._get_gs_value(key)

    def __setitem__(self, key, val):
        """Set the 12-bit PWM value (0-4095) for the specified channel (0-23).
        If auto_write is enabled (the default) then the chip PWM state will
        immediately be updated too, otherwise you must call write to update
        the chip with the new PWM state.
        """
        assert 0 <= key <= 23
        assert 0 <= val <= 4095
        self._set_gs_value(key, val)
