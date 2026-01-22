import picarx_improved
from motions import vector,k_turn, parallel_park,line_following
import time

px = picarx_improved.Picarx()

maneuvers = {
    "kl": k_turn,
    "k": k_turn,
    "p": parallel_park,
    "l": line_following
}

while True:
    request = input("Enter command (q to quit): ")

    if request == "q":
        print("exiting...")
        break

    if request in maneuvers:
        maneuvers[request]()
    else:
        print("not a valid command")