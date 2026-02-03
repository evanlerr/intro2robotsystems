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
        frame = self.picam2.capture_array("main")  # YUV420 → 2D uint8
        gray = frame  # already grayscale, 0–255

        threshold_value = 155
        _, thresh = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)

        h, w = thresh.shape
        row = h -2
        center_size = 70
        cx = w // 2

        #band = 10
        #rows = thresh[row-band : row+band, :]

        center_region = thresh[row, cx - center_size//2 : cx + center_size//2]
        left_region   = thresh[row, cx - center_size//2 - 50 : cx - center_size//2]
        right_region  = thresh[row, cx + center_size//2 : cx + center_size//2 +50]
        left_avg   = int(left_region.mean()   * 1000 / 255)
        center_avg = int(center_region.mean() * 1000 / 255)
        right_avg  = int(right_region.mean()  * 1000 / 255)
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
        values = self.interpreter.state(self.sensor.readval())
        print(f"raw vals: {self.sensor.readval()},output value: {values}, {self.over_left}, {self.over_right}, {self.time_over_line}")
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
interpret_table = interpreter(dark=700, light=1050, polarity="dark") # todo - add a threshold to make the image binary
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
