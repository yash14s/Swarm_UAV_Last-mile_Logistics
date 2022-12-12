from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import numpy as np
from pymavlink import mavutil

#Define number of drones
N = 4
#Define the adjacency matrix
A = [[0,1,1,1],[1,0,1,1],[1,1,0,1],[1,1,1,0]]
#Define the weight matrix
W = [[0,2,2*2**0.5,2],[2,0,2,2*2**0.5],[2*2**0.5,2,0,2],[2,2*2**0.5,2,0]]
#Define loop_delay
dt = 0.05

global Drone
Drone = np.zeros(N)
Drone = list(Drone)

def connect_to_python():
    IP_address = 'udp:127.0.0.1'
    port = 14550
    for i in range(N):
        connection_string = IP_address + ':' + str(port)
        Drone[i] = connect(connection_string, wait_ready=False)
        print("Drone",i+1,"is CONNECTED")
        port += 10

def disconnect_to_python():
    for i in range(N):
        Drone[i].close()
        print("Drone",i+1,"is DISCONNECTED")

def arm(drone):
    while drone.is_armable!=True:
        print("Undergoing pre-arm checks")
        time.sleep(1)
    print('Drone is now armable')
    drone.mode=VehicleMode('GUIDED')
    while drone.mode!='GUIDED':
        print("Waiting to enter GUIDED flight mode")
        time.sleep(1)
    print("Drone is now in GUIDED MODE.")

    drone.armed=True
    while drone.armed==False:
        print("Waiting for Drone to become armed")
        time.sleep(1)
    print("Caution! Drone is ARMED!")

def disarm():
    print("Proceeding to disarm")
    for i in range(N):
        Drone[i].armed = False
        while(Drone[i].armed==True):
            time.sleep(1)
    print("Drones are DISARMED")

def takeoff(drone, targetHeight):
    drone.simple_takeoff(targetHeight)
    while True:
        altitude = drone.location.global_relative_frame.alt
        print("Current Altitude:",altitude)
        if altitude>=.91*targetHeight:
            break
        time.sleep(1)
    print("Reached target altitude")

def land():
    for i in range(N):
        Drone[i].mode=VehicleMode("LAND")
        while Drone[i].mode != 'LAND':
            print("Waiting for drone to enter LAND mode")
            time.sleep(1)
        print("Drone in LAND mode")
    disarm()

def send_global_ned_velocity(drone,vx,vy,vz):
    msg = drone.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_LOCAL_NED,0b0000111111000111,0,0,0,vx,vy,vz,0,0,0,0,0)
    drone.send_mavlink(msg)
    drone.flush()

def haversine_distance(latitude1, longitude1, latitude2, longitude2):
    latitude1 = math.radians(latitude1)
    latitude2 = math.radians(latitude2)
    longitude1 = math.radians(longitude1)
    longitude2 = math.radians(longitude2)
    dlon = longitude2 - longitude1
    dlat = latitude2 - latitude1
    a = math.sin(dlat/2)**2 + math.cos(latitude1) * math.cos(latitude2) * math.sin(dlon/2)**2
    c = 2 * math.asin(a**0.5)
    r = 6371000
    return c*r

def speed_bound(speed):
    #Function to ensure speed doesn't cross a threshold
    threshold = 0.5
    if speed > threshold:
        return threshold
    return speed

def leader_rigid_formation(A, W):
    print("Executing Algorithm")
    #Initialize position arrays
    x = np.zeros(N)
    y = np.zeros(N)
    z = np.zeros(N)
    
    #Define gains k1, k2, k3
    #k1 compensates for small difference value in latitude and longitude
    #k2 is the rendezvous gain
    #k3 is gain for rigid_weights
    k1 = 10**4
    k2 = 0.75
    k3 = 0.4

    #Counter for iterations
    no_of_iterations = 0

    while(True):
        no_of_iterations += 1
        print(no_of_iterations)
        #Before each iteration, set velocity to zero
        x_dot = np.zeros(N)
        y_dot = np.zeros(N)
        z_dot = np.zeros(N)

        #Compute position
        for i in range(N):
            x[i] = Drone[i].location.global_relative_frame.lat
            y[i] = Drone[i].location.global_relative_frame.lon
            z[i] = Drone[i].location.global_relative_frame.alt

        #Compute velocity as per the control law. Z points downward
        # Drone 1 is the leader
        #Define the control law for the followers:
        #Rigid formation in the X-Y Plane, Rendezvous about the Z-axis
        for i in range(1, N):
            for j in range(N):
                rigid_weight = k3 * (haversine_distance(x[i],y[i],x[j],y[j]) - W[i][j])
                x_dot[i] -= A[i][j] * rigid_weight * k1 * (x[i] - x[j])
                y_dot[i] -= A[i][j] * rigid_weight * k1 * (y[i] - y[j])
                z_dot[i] -= -(A[i][j] * k2 * (z[i] - z[j]))

        #Send velocity command to each drone
        for i in range(1,N):
            send_global_ned_velocity(Drone[i], speed_bound(x_dot[i]), speed_bound(y_dot[i]), speed_bound(z_dot[i]))

        time.sleep(dt)

#Mission:
connect_to_python()
for i in range(N):
    print("Arming Drone",i+1)
    arm(Drone[i])
    print("Takeoff initiated for Drone",i+1)
    takeoff(Drone[i], 1)

time.sleep(1)
leader_rigid_formation(A, W)
land()
