all: upload

#flash:
#	sudo ./BOSSA-1.8/bin/bossac -e -w -v -R ./adafruit-circuitpython-feather_m0_rfm9x-3.0.0.bin

upload:
	cp -r src/* /media/brennen/CIRCUITPY/
	sync
