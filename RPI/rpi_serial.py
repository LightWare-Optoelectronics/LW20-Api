#-------------------------------------------------------------------------
# LightWare SF30 Serial Python Example
#-------------------------------------------------------------------------

# NOTE: Still not reliable due to byte order switching.

import time
import serial

laser = serial.Serial("/dev/ttyUSB0", baudrate=230400, timeout=0.01)

print "Lw20 Serial Sample"

state = 0
reading_count = 0
reading_avg = 0
byte_l = -1
byte_h = -1

timer = time.clock() + 1.0

while True:
	time.sleep(1)

	laser.write("?\r")

	buff = laser.read(4096);
	print "Recv " + str(len(buff))