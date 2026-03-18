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

## BELOW IS MQTT SETUP CODE ## 
broker_proc = None
try: 
    import socket
    import tempfile
    import subprocess
    import paho.mqtt.client as mqtt

    host = "0.0.0.0"
    port = 1883
    topic = "robot/state"

    def get_local_ip():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # No packets are sent; this only reveals the preferred outbound IP.
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
        except OSError:
            return "127.0.0.1"
        finally:
            sock.close()

    config_text = f"""listener {port} {host}
allow_anonymous true
"""
    config_file = tempfile.NamedTemporaryFile(mode="w", suffix=".conf", delete=False)
    config_file.write(config_text)
    config_file.flush()
    config_file.close()

    broker_proc = subprocess.Popen(["mosquitto", "-c", config_file.name])
    time.sleep(1)

    advertised_ip = get_local_ip()
    print(f"MQTT broker listening on mqtt://{advertised_ip}:{port}")
    print(f"Publishing timestamps to topic '{topic}'")

    client = mqtt.Client()

    client.connect("127.0.0.1", port, 60) # IF YOU NEED TO LOCALLY CONNECT

    # while True:
    #     current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    #     client.publish(topic, current_time)
    #     print(f"Published: {current_time}")
    #     time.sleep(1)

except FileNotFoundError:
    print("mosquitto is not installed. Install it with: brew install mosquitto")


def ultrasonic_sensor_function():
    vals2 = px.ultrasonic.read()
    print(f"Ultrasonic sensor value: {vals2}\n")
    if vals2 == -2:
        return 1000
    return vals2

def grayscale_sensor_function():
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
    if vals is 0 or vals is None:
        print("no sensor values to interpret")
        return (0,0,0)
        #return None
    dark = 100
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

def controller_function(statevals,statevars,ultrasonic_vals,self_busvars):
    print(f"non-Controlled state values: {statevals}\n")
    if statevals is 0 or (isinstance(statevals,tuple) and all (v is 0 for v in statevals)) or statevals is None:
        print("no state values to control on")
        print("statevars: ", statevars)
        return statevars
    over_left = statevars.get("over_left", False)
    over_right = statevars.get("over_right", False)
    left_count = statevars.get("left_count", 0)
    right_count = statevars.get("right_count", 0)
    time_over_line = statevars.get("time_over_line", 0)

    drivecomm = self_busvars.get("drive_comm", 1)
    drivecomm = 1 if drivecomm is None else drivecomm

    gain = 30
    Ldist = statevals[0]
    Cdist = statevals[1]
    Rdist = statevals[2]
    # Expect ultrasonic value as fourth element in statevals, or None
    ultrasonic_val = ultrasonic_vals if ultrasonic_vals is not None else None

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

    print(f"Ultrasonic sensor value (controller): {ultrasonic_val}")
    obstacle_threshold = 15  # cm, adjust as needed
    speed = 0
    if drivecomm == 1:
        if ultrasonic_val is not None and ultrasonic_val < obstacle_threshold:
            print("Obstacle detected! Stopping movement.")
            px.forward(0)
            speed = 0
            time.sleep(0.2)
        else:
            if Cdist < 0.2 and time_over_line > 25:
                px.set_dir_servo_angle(-turn)
                #px.set_cam_pan_angle(-turn)
                px.backward(20)
                time.sleep(.4)
                speed = -20
            else:
                px.set_dir_servo_angle(turn)
                #px.set_cam_pan_angle(turn)
                px.forward(20)
                time.sleep(.05)
                speed = 20

    new_statevars = {
        "over_left": over_left,
        "over_right": over_right,
        "left_count": left_count,
        "right_count": right_count,
        "time_over_line": time_over_line
    }

    drivevars = {"turn": turn, "speed": speed}

    return new_statevars,drivevars

def comms_out(self_busvars,sensor,drive):
    try:
        loop_count = self_busvars.get("loop_count", 0) if self_busvars is not None else 0
        loop_count += 1
        print(f"communicator output function called ({loop_count})")

        ultrasonic_val = sensor if sensor is not None else None
        speed = drive.get("speed", 0) if isinstance(drive, dict) else 1

        stoptime = self_busvars.get("stoptime", 0) if self_busvars is not None else 0
        armdist = self_busvars.get("armdist", None) if self_busvars is not None else None
        drivecomm = self_busvars.get("drive_comm", 1) if self_busvars is not None else 1
        vicstate = self_busvars.get("vicstate", 0) if self_busvars is not None else 0
        armtime = self_busvars.get("armtime", 0) if self_busvars is not None else 0
        print(f"\n CURRENT VEHICLE STATE: {vicstate}, stoptime: {stoptime}, armdist: {armdist}, drivecomm: {drivecomm}, armtime: {armtime}, ultrasonic_val: {ultrasonic_val}, speed: {speed}\n ")
        if vicstate == 3:
            drivecomm = 0
            time.sleep(4)
            if ultrasonic_val is not None and ultrasonic_val > 30:
                drivecomm = 1
                vicstate = 0
                stoptime = 0
                armtime = 0
                armdist = None

        if vicstate == 2:
            armtime = armtime + 1
            if armdist is not None and ultrasonic_val is not None and armdist < 20:
                if armtime > 2 and ultrasonic_val < armdist-1:
                    vicstate = 3

        if vicstate == 1:
            if armdist is None:
                armdist = ultrasonic_val
                #client.publish(topic, f"detected at distance: {armdist} cm")
            vicstate = 2
            armtime = 0

        if speed == 0:
            stoptime += 1
            if stoptime > 6 and vicstate == 0:
                vicstate = 1
        else:
            stoptime = 0
            vicstate = 0
            armdist = None
            armtime = 0
        try:
            client.publish(topic, vicstate)
        except Exception as err:
            print(f"MQTT publish failed: {err}")

        return {
            "stoptime": stoptime,
            "armdist": armdist,
            "drive_comm": drivecomm,
            "vicstate": vicstate,
            "armtime": armtime,
            "loop_count": loop_count,
        }
    except Exception as err:
        print(f"comms_out crashed: {err}")
        # Keep state loop alive even if one cycle fails.
        return self_busvars if isinstance(self_busvars, dict) else {
            "stoptime": 0,
            "armdist": None,
            "drive_comm": 1,
            "vicstate": 0,
            "armtime": 0,
            "loop_count": 0,
        }

sensor_bus = Bus(name="GrayscaleSensorBus")
sensor2_bus = Bus(name="UltrasonicSensorBus")
state_bus = Bus(name="StateBus")
drive_bus = Bus(name="DriveBus")
self_bus = Bus(initial_message={"stoptime": 0, "armdist": None, "drive_comm": 1, "vicstate": 0, "armtime": 0, "loop_count": 0}, name="SelfBus")
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
    try:
        picam2.stop()
    except Exception:
        pass
    config = picam2.create_still_configuration(main={
    "size": (320, 240), 
    "format": "YUV420"
    })
    picam2.configure(config)
    picam2.start()
    #self.picam2.set_controls({"AeEnable": False, "AwbEnable": False,"ExposureTime": 1000, "AnalogueGain": 10.0})
    picam2.set_controls({"AeEnable": True, "AwbEnable": False})


    sensor = Producer(grayscale_sensor_function, output_buses=sensor_bus, delay=0.02, name="GrayscaleSensor")
    sensor2 = Producer(ultrasonic_sensor_function, output_buses=sensor2_bus, delay=0.1, name="UltrasonicSensor")
    interpreter = ConsumerProducer(interpret_function, input_buses=sensor_bus, output_buses=state_bus, delay=0.02, name="Interpreter")
    controller = ConsumerProducer(controller_function,input_buses=(state_bus, state_var_bus,sensor2_bus),output_buses=(state_var_bus, drive_bus),delay=0.02,name="Controller")

    try:
        runConcurrently([sensor,sensor2, interpreter, controller])
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")
    finally:        
        px.stop()
        if broker_proc is not None:
            try:
                if broker_proc.poll() is None:
                    broker_proc.terminate()
            except Exception:
                pass
        print("stop and exit")

def gsline_comms():
    try:
        picam2.stop()
    except Exception:
        pass
    config = picam2.create_still_configuration(main={
    "size": (320, 240), 
    "format": "YUV420"
    })
    picam2.configure(config)
    picam2.start()
    #self.picam2.set_controls({"AeEnable": False, "AwbEnable": False,"ExposureTime": 1000, "AnalogueGain": 10.0})
    picam2.set_controls({"AeEnable": True, "AwbEnable": False})

    sensor = Producer(grayscale_sensor_function, output_buses=sensor_bus, delay=0.02, name="GrayscaleSensor")
    sensor2 = Producer(ultrasonic_sensor_function, output_buses=sensor2_bus, delay=0.1, name="UltrasonicSensor")
    interpreter = ConsumerProducer(interpret_function, input_buses=sensor_bus, output_buses=state_bus, delay=0.02, name="Interpreter")
    controller = ConsumerProducer(controller_function,input_buses=(state_bus, state_var_bus,sensor2_bus,self_bus),output_buses=(state_var_bus, drive_bus),delay=0.02,name="Controller")
    communicator = ConsumerProducer(comms_out,input_buses=(self_bus ,sensor2_bus, drive_bus),output_buses=self_bus, delay=.05, name="Communicator")

    try:
        runConcurrently([sensor,sensor2, interpreter, controller, communicator])
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")
    finally:        
        px.stop()
        if broker_proc is not None:
            try:
                if broker_proc.poll() is None:
                    broker_proc.terminate()
            except Exception:
                pass
        print("stop and exit")


def mqtt_test():
    broker_proc = None
    try: 
        import socket
        import tempfile
        import subprocess
        import paho.mqtt.client as mqtt

        host = "0.0.0.0"
        port = 1883
        topic = "robot/time"

        def get_local_ip():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                # No packets are sent; this only reveals the preferred outbound IP.
                sock.connect(("8.8.8.8", 80))
                return sock.getsockname()[0]
            except OSError:
                return "127.0.0.1"
            finally:
                sock.close()

        config_text = f"""listener {port} {host}
allow_anonymous true
"""
        config_file = tempfile.NamedTemporaryFile(mode="w", suffix=".conf", delete=False)
        config_file.write(config_text)
        config_file.flush()
        config_file.close()

        broker_proc = subprocess.Popen(["mosquitto", "-c", config_file.name])
        time.sleep(1)

        advertised_ip = get_local_ip()
        print(f"MQTT broker listening on mqtt://{advertised_ip}:{port}")
        print(f"Publishing timestamps to topic '{topic}'")

        client = mqtt.Client()

        client.connect("127.0.0.1", port, 60)

        while True:
            current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            client.publish(topic, current_time)
            print(f"Published: {current_time}")
            time.sleep(1)

    except FileNotFoundError:
        print("mosquitto is not installed. Install it with: brew install mosquitto")
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")
    finally:
        if broker_proc is not None:
            try:
                if broker_proc.poll() is None:
                    broker_proc.terminate()
            except Exception:
                pass
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
