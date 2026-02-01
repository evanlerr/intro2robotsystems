from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput
import http.server
import socketserver
import threading

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())
picam2.start()

encoder = MJPEGEncoder()
output = FileOutput("tcp://0.0.0.0:8000")
picam2.start_recording(encoder, output)

print("Open http://<pi-ip>:8000 in your browser")

try:
    while True:
        pass
except KeyboardInterrupt:
    picam2.stop_recording()
    picam2.stop()
