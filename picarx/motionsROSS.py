import sys,os,picarx_improved,cv2
from sim_robot_hat import config
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from picarx_improved import Picarx
from picamera2 import Picamera2, Preview
#from sim_robot_hat import Greyscale_Module
import time
#import concurrent.futures
#from readerwriterlock import rwlock
from rossros import Bus, Producer, ConsumerProducer, Consumer, runConcurrently

#px = Picarx()
from motions import px
from motions import picam2


def grayscale_sensor_function():
    global px
    vals = px.grayscale.read()
    print(f"Grayscale sensor values: {vals}\n")
    return vals

# def camera_init():
#         global picam2
#         picam2 = Picamera2()
#         config = picam2.create_still_configuration(main={
#             "size": (320, 240), 
#             "format": "YUV420"
#             })
#         picam2.configure(config)
#         picam2.start()
#         #self.picam2.set_controls({"AeEnable": False, "AwbEnable": False,"ExposureTime": 1000, "AnalogueGain": 10.0})
#         picam2.set_controls({"AeEnable": True, "AwbEnable": False})
#         global initcomplete
#         initcomplete = True

def camera_sensor_function():
    frame = picam2.capture_array("main")  # YUV420
    h, w = 240,320
    gray = frame[:h,:w]
    # Apply a binary threshold to the grayscale image
    threshold_value = 128
    _, thresh = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY_INV)
    # Calculate the average pixel value in the left, center, and right regions
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

def interpret_function(vals):
    print(f"non-Interpreted sensor values: {vals}\n")
    if vals is None:
        return None
    dark = 200
    light = 800
    polarity = "dark"
    Lval = vals[0]
    Cval = vals[1]
    Rval = vals[2]

    Ldist = abs(Lval - dark)/(light - dark)
    Cdist = abs(Cval - dark)/(light - dark)
    Rdist = abs(Rval - dark)/(light - dark)
    if polarity == "dark":
        Ldist = 1 - Ldist
        Cdist = 1 - Cdist
        Rdist = 1 - Rdist
    return (Ldist,Cdist,Rdist)

def controller_function(statevals,statevars):
    print(f"non-Controlled state values: {statevals}\n")
    if statevals is None:
        return
    
    over_left = statevars.get("over_left", False)
    over_right = statevars.get("over_right", False)
    left_count = statevars.get("left_count", 0)
    right_count = statevars.get("right_count", 0)
    time_over_line = statevars.get("time_over_line", 0)

    gain = 1
    Ldist = statevals[0]
    Cdist = statevals[1]
    Rdist = statevals[2]
    turn = 0

    left_on_line  = (Ldist > 0.8) and (Ldist > Rdist)
    right_on_line = (Rdist > 0.8) and (Rdist > Ldist)

    if left_on_line:
        print("Left sensor on line")
        over_right = False
        time_over_line = 0
    if right_on_line:
        print("Right sensor on line")
        over_left = False
        time_over_line = 0

    CONFIRM = 20

    if left_on_line:
        left_count += 1
    else:
        left_count = 0

    if right_on_line:
        right_count += 1
    else:
        right_count = 0

    

    if not (over_left or over_right):
        if left_count >= CONFIRM and not over_right:
            over_left = True
        elif right_count >= CONFIRM and not over_left:
            over_right = True


    if over_left or over_right:
        time_over_line += 1

    if Ldist > Rdist:
        turn = -1 * gain * (Ldist - Rdist)
    else:
        turn = gain * (Rdist - Ldist)

    if over_left:
        turn = -30
    if over_right:
        turn = 30


    if Cdist < 0.2 and time_over_line > 25:
        px.set_dir_servo_angle(-turn)
        #px.set_cam_pan_angle(-turn)
        px.backward(20)
        time.sleep(.4)
    else:
        px.set_dir_servo_angle(turn)
        #px.set_cam_pan_angle(turn)
        px.forward(20)
        time.sleep(.05)

    new_statevars = {
        "over_left": over_left,
        "over_right": over_right,
        "left_count": left_count,
        "right_count": right_count,
        "time_over_line": time_over_line
    }

    return None,new_statevars

sensor_bus = Bus()
state_bus = Bus()
state_var_bus = Bus(initial_message={"over_left": False, "over_right": False, "left_count": 0, "right_count": 0, "time_over_line": 0}, name="StateVarBus")

# sensegs = Sensor(px.grayscale.pins,px.grayscale,sensor_bus)
# sensevis = DigiSensor(sensor_bus,delay=0.02)
# interpret = interpreter(dark=200, light=800, inbus = sensor_bus, outbus = state_bus, polarity="dark", delay=0.02) # todo - add a threshold to make the image binary
# interpret_table = interpreter(dark=255, light=0, polarity="dark",inbus=sensor_bus,outbus=state_bus, delay=0.02) # todo - add a threshold to make the image binary
# gsfollow_controller = controller(1,state_bus,delay=0.02)
# clfollow_controller = controller(40,state_bus,delay=0.02)

#camera_init()

#global picam2

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
    config = picam2.create_still_configuration(main={
    "size": (320, 240), 
    "format": "YUV420"
    })
    picam2.configure(config)
    picam2.start()
    #self.picam2.set_controls({"AeEnable": False, "AwbEnable": False,"ExposureTime": 1000, "AnalogueGain": 10.0})
    picam2.set_controls({"AeEnable": True, "AwbEnable": False})


    sensor = Producer(grayscale_sensor_function, output_buses=sensor_bus, delay=0.05, name="GrayscaleSensor")
    interpreter = ConsumerProducer(interpret_function, input_buses=sensor_bus, output_buses=state_bus, delay=0.05, name="Interpreter")
    controller = ConsumerProducer(controller_function,input_buses=(state_bus, state_var_bus),output_buses=(None, state_var_bus),delay=0.05,name="Controller")

    try:
        runConcurrently([sensor, interpreter, controller])
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")
    finally:        
        px.stop()
        print("stop and exit")

# def gsline_following():
#     try:
#             with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
#                 executor.submit(sensegs.readval)
#                 executor.submit(interpret.state)
#                 executor.submit(gsfollow_controller.control_loop)
#                 while True:
#                     time.sleep(1)
#     except KeyboardInterrupt:
#         print("\nKeyboardInterrupt: stop and exit")
#     finally:
#         px.stop()
#         print("stop and exit")

# def cameraline_following():
#     px.set_cam_pan_angle(0)
#     px.set_cam_tilt_angle(-30)
#     try:
#             with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
#                 executor.submit(sensevis.readval)
#                 executor.submit(interpret.state)
#                 executor.submit(clfollow_controller.control_loop)
#                 while True:
#                     time.sleep(1)
#     except KeyboardInterrupt:
#         print("\nKeyboardInterrupt: stop and exit")
#     finally:
#         px.stop()
#         print("stop and exit")


# def cameraline_calibrate():
#     px.set_cam_pan_angle(0)
#     px.set_cam_tilt_angle(-30)
#     time.sleep(2)
#     vals = sensevis.calibrate()
#     print(f"Camera line following calibration values: {vals}")
