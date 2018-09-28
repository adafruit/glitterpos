"""
Main program loop for Glitter Positioning System.

https://github.com/adafruit/glitterpos

* Author(s): Brennen Bearnes, Limor Fried
"""

from glitterpos import GlitterPOS
gp = GlitterPOS()
while True:
    gp.advance_frame()
