import sys,os,picarx_improved,cv2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from picarx_improved import Picarx
from picamera2 import Picamera2, Preview
#from sim_robot_hat import Greyscale_Module
import time
import concurrent.futures
from readerwriterlock import rwlock

class Bus(object):
    def __init__(self):
        self.data = {}
        self.lock = rwlock.RWLockWriteD()


    def write(self, key, value):
        with self.lock.gen_wlock():
            self.data[key] = value

    def read(self, key):
        with self.lock.gen_rlock():
            return self.data.get(key, None)

class Sensor(object):
    def __init__(self,pins,sensor,bus,delay=0.05):
        self.pins = pins
        self.sensor = sensor
        self.bus = bus
        self.delay = delay

    def readval(self):
        while True:
            self.valouts = self.sensor.read()
            self.bus.write("sensor", self.valouts)
            time.sleep(self.delay)
    
class DigiSensor(object):
    def __init__(self,bus,delay=0.05):
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={
            "size": (320, 240), 
            "format": "YUV420"
            })
        self.picam2.configure(config)
        self.picam2.start()
        #self.picam2.set_controls({"AeEnable": False, "AwbEnable": False,"ExposureTime": 1000, "AnalogueGain": 10.0})
        self.picam2.set_controls({"AeEnable": True, "AwbEnable": False})
        time.sleep(2)  # Allow camera to warm up
        self.bus = bus
        self.delay = delay

    def readval(self):
        while True:
            frame = self.picam2.capture_array("main")  # YUV420 → 2D uint8
            h, w = 240,320
            gray = frame[:h,:w]

            threshold_value = 130
            _, thresh = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY_INV)

            h, w = thresh.shape
            rows = slice(h-25, h)
            center_size = 70
            cx = w // 2

            #band = 10
            #rows = thresh[row-band : row+band, :]

            center_region = thresh[rows, cx - center_size//2 : cx + center_size//2]
            left_region   = thresh[rows, cx - center_size//2 - 100 : cx - center_size//2]
            right_region  = thresh[rows, cx + center_size//2 : cx + center_size//2 +100]
            left_avg   = int(left_region.mean())
            center_avg = int(center_region.mean())
            right_avg  = int(right_region.mean())
            cv2.imwrite("cam_gray.png", gray)
            cv2.imwrite("cam_thresh.png", thresh)
            #cv2.waitKey(1)
            self.bus.write("sensor", (left_avg, center_avg, right_avg))
            time.sleep(self.delay)

    def calibrate(self):
        frame = self.picam2.capture_array("main")
        h, w = 240, 320
        gray = frame[:h, :w]
        threshold_value = 130
        _, thresh = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY_INV)
        h, w = thresh.shape
        rows = slice(h-25, h)
        center_size = 70
        cx = w // 2
        center_region = thresh[rows, cx - center_size//2 : cx + center_size//2]
        left_region   = thresh[rows, cx - center_size//2 - 100 : cx - center_size//2]
        right_region  = thresh[rows, cx + center_size//2 : cx + center_size//2 +100]
        left_avg   = int(left_region.mean())
        center_avg = int(center_region.mean())
        right_avg  = int(right_region.mean())
        return (left_avg, center_avg, right_avg)


    
class interpreter(object):
    def __init__(self, dark,light,polarity,inbus,outbus,delay = 0.05):
        self.dark = dark
        self.light = light
        self.polarity = polarity
        self.inbus = inbus
        self.outbus = outbus
        self.delay = delay

    def state(self):
        while True:

            vals = self.inbus.read("sensor")

            if vals is None:
                time.sleep(self.delay)
                continue
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

            self.outbus.write("state", (Ldist,Cdist,Rdist))

            time.sleep(self.delay)
        
class controller(object):
    def __init__(self, gain,bus,delay=0.05):
        self.left_count = 0
        self.right_count = 0
        self.over_left = False
        self.over_right = False
        self.time_over_line = 0
        self.bus = bus
        self.delay = delay
        self.gain = gain

    def control_loop(self):
        while True:

            values = self.bus.read("state") # wait for new state to be available
            if values is None:
                time.sleep(self.delay)
                continue
            #raw = self.sensor.readval()
            #values = self.interpreter.state(raw)
            print(f"output value: {values}, {self.over_left}, {self.over_right}, {self.time_over_line}")
            gain = self.gain
            Ldist = values[0]
            Cdist = values[1]
            Rdist = values[2]
            turn = 0

            left_on_line  = (Ldist > 0.8) and (Ldist > Rdist)
            right_on_line = (Rdist > 0.8) and (Rdist > Ldist)

            if left_on_line:
                print("Left sensor on line")
                self.over_right = False
                self.time_over_line = 0
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
                turn = -1 * gain * (Ldist - Rdist)
            else:
                turn = gain * (Rdist - Ldist)

            if self.over_left:
                turn = -30
            if self.over_right:
                turn = 30


            if Cdist < 0.2 and self.time_over_line > 25:
                px.set_dir_servo_angle(-turn)
                #px.set_cam_pan_angle(-turn)
                px.backward(20)
                time.sleep(.4)
            else:
                px.set_dir_servo_angle(turn)
                #px.set_cam_pan_angle(turn)
                px.forward(20)
                time.sleep(.05)

            time.sleep(self.delay)

sensor_bus = Bus()
state_bus = Bus()

px = Picarx()
#picam2 = Picamera2()
sensegs = Sensor(px.grayscale.pins,px.grayscale,sensor_bus)
sensevis = DigiSensor(sensor_bus,delay=0.02)
interpret = interpreter(dark=200, light=800, inbus = sensor_bus, outbus = state_bus, polarity="dark", delay=0.02) # todo - add a threshold to make the image binary
interpret_table = interpreter(dark=255, light=0, polarity="dark",inbus=sensor_bus,outbus=state_bus, delay=0.02) # todo - add a threshold to make the image binary
gsfollow_controller = controller(1,state_bus,delay=0.02)
clfollow_controller = controller(40,state_bus,delay=0.02)

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
            with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
                executor.submit(sensegs.readval)
                executor.submit(interpret.state)
                executor.submit(gsfollow_controller.control_loop)
                while True:
                    time.sleep(1)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")
    finally:
        px.stop()
        print("stop and exit")

def cameraline_following():
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(-30)
    try:
            with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
                executor.submit(sensevis.readval)
                executor.submit(interpret.state)
                executor.submit(clfollow_controller.control_loop)
                while True:
                    time.sleep(1)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")
    finally:
        px.stop()
        picam2.close()
        print("stop and exit")


def cameraline_calibrate():
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(-30)
    time.sleep(2)
    vals = sensevis.calibrate()
    print(f"Camera line following calibration values: {vals}")
