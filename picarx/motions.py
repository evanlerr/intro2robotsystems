import sys,os,picarx_improved
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from picarx_improved import Picarx
#from sim_robot_hat import Greyscale_Module
import time

class Sensor(object):
    def __init__(self,pins,sensor):
        self.pins = pins
        self.sensor = sensor

    def readval(self):
        self.valouts = self.sensor.read()
        return self.valouts
    
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

    def control_loop(self):
        values = self.interpreter.state(self.sensor.readval())
        print(f"output value: {values}, {self.over_left}, {self.over_right}, {self.time_over_line}")
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
sense = Sensor(px.grayscale.pins,px.grayscale)
interpret = interpreter(dark=200, light=800, polarity="dark")
follow_controller = controller(30,sense,interpret)

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
            follow_controller.control_loop()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")
    finally:
        px.stop()
        print("stop and exit")

def cameraline_following():
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(-30)
    time.sleep(1)
    px.set_cam_tilt_angle(-40)
    time.sleep(1)
