from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import numpy as np

#Define number of drones
N = 3
global Drone
Drone = np.zeros(N)
Drone = list(Drone)

def connect_to_python():
    IP_address = 'tcp:192.168.0.177'
    port = 5762
    for i in range(N):
        connection_string = IP_address + ':' + str(port)
        Drone[i] = connect(connection_string, wait_ready=False)
        print("Drone",i+1,"is CONNECTED")
        port += 10

def disconnect_to_python():
    for i in range(N):
        Drone[i].close()
        print("Drone",i+1,"is DISCONNECTED")

def arm():
    connect_to_python()
    for i in range(N):
        while Drone[i].is_armable!=True:
            print("Drone",i+1,"is Undergoing pre-arm checks")
            time.sleep(1)
        print('Drone',i+1,'is now armable')
        Drone[i].mode=VehicleMode('GUIDED')
        while Drone[i].mode!='GUIDED':
            print("Waiting for Drone",i+1,"to enter GUIDED flight mode")
            time.sleep(1)
        print("Drone",i+1,"is now in GUIDED MODE.")

    for i in range(N):
        Drone[i].armed=True
        while Drone[i].armed==False:
            print("Waiting for Drone",i+1,"to become armed")
            time.sleep(1)
        print("Caution! Drone",i+1,"is ARMED!")
    print("ALL DRONES ARMED")

def disarm():
    print("Proceeding to disarm")
    for i in range(N):
        Drone[i].armed = False
        while(Drone[i].armed==True):
            time.sleep(1)
    print("Drones are DISARMED")

def takeoff(targetHeight):
    arm()
    for i in range(N):
        Drone[i].simple_takeoff(targetHeight)#meters
        while True:
            altitude = Drone[i].location.global_relative_frame.alt
            print("Current Altitude: %f"%altitude)
            if altitude>=.91*targetHeight:
                break
            time.sleep(1)
        print("Drone",i+1,"has reached target altitude")

def land():
    for i in range(N):
        Drone[i].mode=VehicleMode("LAND")
        while Drone[i].mode != 'LAND':
            print("Waiting for drone to enter LAND mode")
            time.sleep(1)
        print("Drone in LAND mode")
    disarm()

def takeoff_land(targetHeight):
    takeoff(targetHeight)
    print("Hovering for 10 seconds")
    time.sleep(10)
    land()
    disconnect_to_python()

takeoff_land(0.2)
