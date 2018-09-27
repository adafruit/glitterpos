"""Configuration values for the Glitter Positioning System."""

# id should be a unique integer value for each box:
MY_ID = 0

# Compass calibration values.  From the CircuitPython REPL, use `import
# calibrate` to find values for MAG_MIN and MAG_MAX:
MAG_MIN = [-0.1883, -0.16002, -0.53634]
MAG_MAX = [0.5887, 0.72618, 0.19474]

# Magnetic North - should be customized for your rough location:
DECLINATION_RAD = 0.1451 # Declination for Boulder, CO
