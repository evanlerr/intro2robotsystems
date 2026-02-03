import sys,os,picarx_improved,cv2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from picarx_improved import Picarx
from picamera2 import Picamera2, Preview
#from sim_robot_hat import Greyscale_Module
import time

class Sensor(object):
    def __init__(self,pins,sensor):
        self.pins = pins
        self.sensor = sensor

    def readval(self):
        self.valouts = self.sensor.read()
        return self.valouts
    
class DigiSensor(object):
    def __init__(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={
            "size": (320, 240), 
            "format": "YUV420"
            })
        self.picam2.configure(config)
        self.picam2.start()
        self.picam2.set_controls({"AeEnable": False, "AwbEnable": False,"ExposureTime": 10000, "AnalogueGain": 1.0})
        time.sleep(2)  # Allow camera to warm up

    def readval(self):
        # Capture image and ensure a single-channel grayscale image
        frame = self.picam2.capture_array("main")
        if frame is None:
            return 0, 0, 0
        # If frame has multiple channels, convert to grayscale
        if len(frame.shape) == 3 and frame.shape[2] in (3, 4):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        # Use Otsu to pick a good threshold automatically and invert so the line is white
        try:
            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        except Exception:
            # Fallback to a fixed threshold if Otsu fails
            _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        h, w = thresh.shape[:2]
        rows = slice(max(0, h-20), h)  # use last 20 rows for robustness
        center_size = max(20, w // 5)
        cx = w // 2

        # Safely compute left/center/right regions with bounds checks
        c1 = max(0, cx - center_size // 2)
        c2 = min(w, cx + center_size // 2)
        l1 = max(0, c1 - center_size)
        l2 = c1
        r1 = c2
        r2 = min(w, c2 + center_size)

        center_region = thresh[rows, c1:c2]
        left_region = thresh[rows, l1:l2]
        right_region = thresh[rows, r1:r2]

        # Handle empty slices
        def safe_mean(arr):
            if arr.size == 0:
                return 0
            return int(arr.mean())

        left_avg = safe_mean(left_region)
        center_avg = safe_mean(center_region)
        right_avg = safe_mean(right_region)
        return left_avg, center_avg, right_avg




    
class interpreter(object):
    def __init__(self, dark,light,polarity):
        self.dark = dark
        self.light = light
        self.polarity = polarity

    def state(self, vals):
        Lval = vals[0]
        Cval = vals[1]
        Rval = vals[2]
        
        Ldist = abs(Lval - self.dark)/(self.light - self.dark)
        Cdist = abs(Cval - self.dark)/(self.light - self.dark)
        Rdist = abs(Rval - self.dark)/(self.light - self.dark)

        if self.polarity == "dark":
            Ldist = 1 - Ldist
            Cdist = 1 - Cdist
            Rdist = 1 - Rdist
        return (Ldist,Cdist,Rdist)
        
class controller(object):
    def __init__(self, gain, sensor, interpreter):
        self.gain = gain
        self.sensor = sensor
        self.interpreter = interpreter
        self.left_count = 0
        self.right_count = 0
        self.over_left = False
        self.over_right = False
        self.time_over_line = 0

    def control_loop(self,gain):
        raw_vals = self.sensor.readval()
        values = self.interpreter.state(raw_vals)
        print(f"raw vals: {raw_vals}, output value: {values}, {self.over_left}, {self.over_right}, {self.time_over_line}")
        self.gain = gain
        Ldist = values[0]
        Cdist = values[1]
        Rdist = values[2]
        turn = 0

        left_on_line = Ldist > 0.8
        if left_on_line:
            print("Left sensor on line")
            self.over_right = False
            self.time_over_line = 0
        right_on_line = Rdist > 0.8
        if right_on_line:
            print("Right sensor on line")
            self.over_left = False
            self.time_over_line = 0

        CONFIRM = 20

        if left_on_line:
            self.left_count += 1
        else:
            self.left_count = 0

        if right_on_line:
            self.right_count += 1
        else:
            self.right_count = 0

        

        if not (self.over_left or self.over_right):
            if self.left_count >= CONFIRM and not self.over_right:
                self.over_left = True
            elif self.right_count >= CONFIRM and not self.over_left:
                self.over_right = True


        if self.over_left or self.over_right:
            self.time_over_line += 1

        if Ldist > Rdist:
            turn = -1 * self.gain * (Ldist - Rdist)
        else:
            turn = self.gain * (Rdist - Ldist)

        if self.over_left:
            turn = -30
        if self.over_right:
            turn = 30


        if Cdist < 0.2 and self.time_over_line > 25:
            px.set_dir_servo_angle(-turn)
            px.backward(20)
            time.sleep(.4)
        else:
            px.set_dir_servo_angle(turn)
            px.forward(20)
            time.sleep(.05)

px = Picarx()
sensegs = Sensor(px.grayscale.pins,px.grayscale)
sensevis = DigiSensor()
interpret = interpreter(dark=200, light=800, polarity="dark")
# Camera produces 0-255 pixel values after thresholding; the line is bright (white) after THRESH_BINARY_INV
interpret_table = interpreter(dark=0, light=255, polarity="light") # camera values are 0-255, polarity 'light' treats bright as on-line
gsfollow_controller = controller(30,sensegs,interpret)
clfollow_controller = controller(30,sensevis,interpret_table)

def vector(angle, speed):
    px.set_dir_servo_angle(angle)
    if speed >= 0:
        px.forward(speed)
    else:
        px.backward(-speed)
    time.sleep(1)
    px.stop()
    

def k_turn(direction="left"):
    if direction == "left":
        x = -1
    else:
        x = 1
    px.set_dir_servo_angle(-30*x)
    px.forward(40)
    time.sleep(2)
    px.set_dir_servo_angle(30*x)
    px.backward(40)
    time.sleep(2)
    px.set_dir_servo_angle(0)
    px.forward(40)
    time.sleep(1)
    px.stop()
    px.set_dir_servo_angle(0)

def parallel_park():
    # px.forward(30)
    # time.sleep(1)
    # px.stop()
    px.backward(30)
    time.sleep(1)
    px.set_dir_servo_angle(30)
    px.backward(30)
    time.sleep(1)
    px.set_dir_servo_angle(0)
    px.backward(30)
    time.sleep(1)
    px.set_dir_servo_angle(-30)
    px.backward(30)
    time.sleep(1)
    px.stop()
    px.set_dir_servo_angle(0)

def gsline_following():
    try:
        while True:
            gsfollow_controller.control_loop(1)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")
    finally:
        px.stop()
        print("stop and exit")

def cameraline_following():
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(-30)
    try:
        while True:
            clfollow_controller.control_loop(50)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")
    finally:
        px.stop()
        print("stop and exit")


def cameraline_calibrate():
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(-30)
    time.sleep(2)
    vals = sensevis.readval()
    print(f"Camera line following calibration values: {vals}")
