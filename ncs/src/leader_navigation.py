from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import numpy as np
from pymavlink import mavutil

N = 4
#Define loop_delay
dt = 0.05

pickup_coords = LocationGlobalRelative(-35.3631937, 149.1650171, 2)
delivery_coords = LocationGlobalRelative(-35.3631852, 149.1639357, 2)

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
 
def navigate(waypoint, nav_speed = 0.5):
    x = np.zeros(N)
    y = np.zeros(N)
    z = np.zeros(N)

    x_dot = np.zeros(N)
    y_dot = np.zeros(N)
    z_dot = np.zeros(N)

    k4 = 0.75

    x[0] = Drone[0].location.global_relative_frame.lat
    y[0] = Drone[0].location.global_relative_frame.lon
    z[0] = Drone[0].location.global_relative_frame.alt
    distance_to_destination_initial = haversine_distance(x[0], y[0], waypoint.lat, waypoint.lon)
    print("Initial Distance:",distance_to_destination_initial)
    while(True):
        #Compute position
        x[0] = Drone[0].location.global_relative_frame.lat
        y[0] = Drone[0].location.global_relative_frame.lon
        z[0] = Drone[0].location.global_relative_frame.alt

        #Compute velocity as per the control law. Z points downward
        #Define control law for the leader:
        x_goal = waypoint.lat
        y_goal = waypoint.lon
        z_goal = waypoint.alt
        heading = math.atan2(y_goal-y[0], x_goal-x[0])
        x_dot[0] = nav_speed * math.cos(heading)
        y_dot[0] = nav_speed * math.sin(heading)
        z_dot[0] = -k4 * (z_goal - z[0])

        #Send velocity command
        send_global_ned_velocity(Drone[0], speed_bound(x_dot[0]), speed_bound(y_dot[0]), speed_bound(z_dot[0]))

        #Current distance to the destination
        distance_to_destination_current = haversine_distance(x[0], y[0], waypoint.lat, waypoint.lon)

        #Terminating condition
        if (distance_to_destination_current < (0.01 * distance_to_destination_initial)):
            print("****Reached Destination!****")
            print("Current Distance:", distance_to_destination_current)
            #Send velocity command
            send_global_ned_velocity(Drone[0], 0, 0, 0)
            break
        time.sleep(dt)

#Mission:
connect_to_python()
for i in range(1):
    print("Arming Drone",i+1)
    arm(Drone[i])
    print("Takeoff initiated for Drone",i+1)
    takeoff(Drone[i], 1)

time.sleep(1)
navigate(pickup_coords)
land()
