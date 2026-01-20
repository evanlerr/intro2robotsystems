import picarx_improved

def j_turn():
    picarx_improved.forward(50)  
    picarx_improved.sleep(2)      
    picarx_improved.stop()

def parallel_park():
    picarx_improved.turn_left(50)  
    picarx_improved.sleep(1.5)      
    picarx_improved.forward(50)     
    picarx_improved.sleep(2)        
    picarx_improved.turn_right(50) 
    picarx_improved.sleep(1.5)      
    picarx_improved.stop()




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