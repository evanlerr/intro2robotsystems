import picarx_improved
from motions import vector,k_turn, parallel_park,gsline_following,cameraline_following,cameraline_calibrate
from motionsROSS import vector as ros_vector, k_turn as ros_k_turn, parallel_park as ros_parallel_park, gsline_following as ros_gsline_following#, cameraline_following as ros_cameraline_following, cameraline_calibrate as ros_cameraline_calibrate
import time

#px = picarx_improved.Picarx()

maneuvers = {
    "kl": k_turn,
    "k": k_turn,
    "p": parallel_park,
    "gl": gsline_following,
    "rgl": ros_gsline_following,
    "cl": cameraline_following,
    #"rcl": ros_cameraline_following,
    "ccal": cameraline_calibrate,
    #"rccal": ros_cameraline_calibrate

}

while True:
    request = input("Enter command (q to quit): ")

    if request == "q":
        print("exiting...")
        break
    if request in maneuvers:
        maneuvers[request]()
    if request[0] == "f":
        if request[1] == "l":
            angle = -int(request[2:4])
        elif request[1] == "r":
            angle = int(request[2:4])
        speed = int(request[4:])
        vector(angle, speed)
        continue
    if request[0] == "r":
        if request[1] == "l":
            angle = -int(request[2:4])
        elif request[1] == "r":
            angle = int(request[2:4])
        speed = -int(request[4:])
        vector(angle, speed)
        continue
    else:
        print("not a valid command")