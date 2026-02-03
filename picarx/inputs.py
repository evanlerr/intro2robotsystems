import picarx_improved
from motions import vector,k_turn, parallel_park,gsline_following,cameraline_following,cameraline_calibrate
import time

#px = picarx_improved.Picarx()

maneuvers = {
    "kl": k_turn,
    "k": k_turn,
    "p": parallel_park,
    "gl": gsline_following,
    "cl": cameraline_following,
    "ccal": cameraline_calibrate

}

while True:
    request = input("Enter command (q to quit): ")

    if request == "q":
        print("exiting...")
        break

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


    if request in maneuvers:
        maneuvers[request]()
    else:
        print("not a valid command")