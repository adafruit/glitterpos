all: select_libs upload

select_libs:
	cp lib/adafruit_rfm9x.py src/lib/
	cp -r lib/adafruit_bus_device/ src/lib/
	cp -r lib/adafruit_gps.py src/lib/
	cp -r lib/neopixel.py src/lib
	cp -r lib/simpleio.py src/lib

#flash:
#	sudo ./BOSSA-1.8/bin/bossac -e -w -v -R ./adafruit-circuitpython-feather_m0_rfm9x-3.0.0.bin

upload: select_libs
	cp -r src/* /media/brennen/CIRCUITPY/
	sync
