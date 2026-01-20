import picarx_improved
from picarx_improved import Picarx
import time

px = Picarx()

def j_turn():
    px.backward(30)
    px.set_dir_servo_angle(20)
    px.forward(30)
    px.set_dir_servo_angle(-20)
    px.forward(30)

def parallel_park():
    px.forward(30)
    time.sleep(1)
    px.stop()
    px.backward(30)
    time.sleep(1)
    px.set_dir_servo_angle(30)
    px.backward(30)
    time.sleep(1)
    px.set_dir_servo_angle(-30)
    px.backward(30)
    time.sleep(1)
    px.stop()
    px.set_dir_servo_angle(0)

    







maneuvers = {
    "j": j_turn,
    "p": parallel_park
}

while True:
    request = input("Enter command: (j for j turn, p for parallel park, q to quit)")

    if request == "q":
        print("exiting...")
        break

    if request in maneuvers:
        maneuvers[request]()
    else:
        print("not a valid command")