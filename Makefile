all: upload

upload:
	cp -r ./*.py /media/brennen/CIRCUITPY/
	cp -r ./lib /media/brennen/CIRCUITPY/
	sync
