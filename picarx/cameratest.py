from picamera2 import Picamera2
import time

picam2 = Picamera2()
picam2.start_preview()   # uses Qt preview window
picam2.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    picam2.stop()
