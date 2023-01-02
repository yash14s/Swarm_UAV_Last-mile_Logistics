from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import numpy as np
from pymavlink import mavutil

#Define number of drones
N = 4

#Define radius of communication (in meters)
R = 6

#Define the adjacency matrix at T = 0
A = [[0,0,1,1],[0,0,1,0],[1,1,0,1],[1,0,1,0]]

#Define the weight matrices
W_square = [[0,2,2*2**0.5,2],[2,0,2,2*2**0.5],[2*2**0.5,2,0,2],[2,2*2**0.5,2,0]]
W_line = [[0,1.5,1.5,2.5],[1.5,0,2.5,3.5],[1.5,2.5,0,1.5],[2.5,3.5,1.5,0]]
W_triangle = [[0,2,2,1.155],[2,0,2,1.155],[2,2,0,1.155],[1.155,1.155,1.155,0]]

#Define loop_delay
dt = 0.05

#Define waypoints
base_coords = LocationGlobalRelative(-35.3632604, 149.1652158, 1)
pickup_coords = LocationGlobalRelative(-35.3631972, 149.1651100, 2)
delivery_coords = LocationGlobalRelative(-35.3634575, 149.1647973, 2)

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

def update_adjacency_matrix(A, x, y):
    for i in range(N):
        for j in range(N):
            if (i != j) and (haversine_distance(x[i], y[i], x[j], y[j]) < R):
                A[i][j] = 1
            else:
                A[i][j] = 0
    return A

def initialize_position():
    x = np.zeros(N)
    y = np.zeros(N)
    z = np.zeros(N)
    return x, y, z

def initialize_velocity():
    x_dot = np.zeros(N)
    y_dot = np.zeros(N)
    z_dot = np.zeros(N)
    return x_dot, y_dot, z_dot

def compute_position(x, y, z):
    for i in range(N):
        x[i] = Drone[i].location.global_relative_frame.lat
        y[i] = Drone[i].location.global_relative_frame.lon
        z[i] = Drone[i].location.global_relative_frame.alt
    return x, y, z

def rendezvous(A):
    print("Adjacency matrix before Rendezvous:", A)
    print("*****Executing Rendezvous*****")
    #Initialize position
    x, y, z = initialize_position()
    #Compute the centroid
    x, y, z = compute_position(x, y, z)
    centroid = [np.mean(x), np.mean(y), np.mean(z)]
    distance_to_centroid = np.zeros(N)
    for i in range(N):
        distance_to_centroid[i] = haversine_distance(centroid[0], centroid[1], x[i], y[i])
    mean_distance_to_centroid_initial = np.mean(distance_to_centroid)

    #Define gain k1, k2, k3
    k1 = 10**4
    k2 = 0.2
    k3 = 1

    while(True):
        #Compute position
        x, y, z = compute_position(x, y, z)

        #Update the Adjacency Matrix:
        A = update_adjacency_matrix(A, x, y)

        #Set velocity to zero
        x_dot, y_dot, z_dot = initialize_velocity()

        #Compute velocity as per the control law. Z points downward
        for i in range(N):
            for j in range(N):
                x_dot[i] -= k2 * A[i][j] * k1 * (x[i] - x[j])
                y_dot[i] -= k2 * A[i][j] * k1 * (y[i] - y[j])
                z_dot[i] -= -k3 * (A[i][j] * (z[i] - z[j]))

        #Send velocity command to each drone
        for i in range(N):
            send_global_ned_velocity(Drone[i], speed_bound(x_dot[i]), speed_bound(y_dot[i]), speed_bound(z_dot[i]))

        #Terminating condition
        terminate = True   
        for i in range(N):
            distance_to_centroid[i] = haversine_distance(centroid[0], centroid[1], x[i], y[i])
            if distance_to_centroid[i] > 2:
                terminate = False
        if terminate:
            print("****Rendezvous Achieved!****")
            #Send velocity command to each drone
            for i in range(N):
                send_global_ned_velocity(Drone[i], 0, 0, 0)
            break

        time.sleep(dt)

    print("Adjacency matrix after Rendezvous:", A)

def is_formation_done(x, y, z):
    x = np.round(x,2)
    y = np.round(x,2)
    z = np.round(z,2)

    cnt = len(np.where(x==0)[0]) + len(np.where(y==0)[0]) + len(np.where(z==0)[0])
    if cnt > 6:
        return True
    else:
        return False
    
def rigid_formation(A, W):
    print("****Commencing Formation****")
    #Initialize position and velocity arrays
    x, y, z = initialize_position()
    
    #Define gains k1, k2, k3
    #k1 compensates for small difference value in latitude and longitude
    #k2 is the rendezvous gain about Z-axis
    #k3 is gain for rigid_weights
    k1 = 10**4
    k2 = 0.75
    k3 = 0.4

    #Counter for tracking instances when velocity becomes very small
    small_velocity_counter = 0
    max_counter_value = 20
    
    while(True):
        #Before each iteration, set velocity to zero
        x_dot, y_dot, z_dot = initialize_velocity()

        #Compute position
        x, y, z = compute_position(x, y, z)

        #Update the adjacency matrix
        A = update_adjacency_matrix(A, x, y)

        #Rigid formation in the X-Y Plane, Rendezvous about the Z-axis
        for i in range(N):
            for j in range(N):
                rigid_weight = k3 * (haversine_distance(x[i],y[i],x[j],y[j]) - W[i][j])
                x_dot[i] -= A[i][j] * rigid_weight * k1 * (x[i] - x[j])
                y_dot[i] -= A[i][j] * rigid_weight * k1 * (y[i] - y[j])
                z_dot[i] -= -(A[i][j] * k2 * (z[i] - z[j]))

        #Send velocity command to each drone
        for i in range(N):
            send_global_ned_velocity(Drone[i], speed_bound(x_dot[i]), speed_bound(y_dot[i]), speed_bound(z_dot[i]))

        #Check if formation is complete
        if is_formation_done(x_dot,y_dot,z_dot):
            small_velocity_counter += 1
        else:
            small_velocity_counter = 0

        if small_velocity_counter == max_counter_value:
            print("Formation done")
            time.sleep(1)
            break
        time.sleep(dt)

def select_leader(waypoint):
    x, y, z = initialize_position()
    #The drone nearest to the waypoint is chosen as the leader
    min_distance = np.inf
    leader = None
    #Compute position
    x, y, z = compute_position(x, y, z)
    for i in range(N):
        distance_to_waypoint = haversine_distance(x[i], y[i], waypoint.lat, waypoint.lon)
        if distance_to_waypoint < min_distance:
            min_distance = distance_to_waypoint
            leader = i        
    print("Leader is Drone ",leader+1)
    return leader

def is_navigation_done(initial_distance, waypoint, x, y):
    current_distance = haversine_distance(waypoint.lat, waypoint.lon, x, y)
    #print("Current Distance:",current_distance)
    if current_distance < 0.1 * initial_distance:
        print("*****Reached*****")
        return True
    return False

def fly_in_formation(waypoint, A, W):
    print("****Flying to Waypoint****")
    #Initialize position and velocity arrays
    x, y, z = initialize_position()
    
    #Define gains k1, k2, k3, k4 and leader's speed
    #k1 compensates for small difference value in latitude and longitude
    #k2 is the rendezvous gain about Z-axis
    #k3 is gain for rigid_weights
    #k4 is leader's altitude control gain
    k1 = 10**4
    k2 = 0.75
    k3 = 3
    k4 = 0.6
    nav_speed = 0.25

    #Choose leader
    leader = select_leader(waypoint)
    
    #Compute initial distance to destination:
    x[leader] = Drone[leader].location.global_relative_frame.lat
    y[leader] = Drone[leader].location.global_relative_frame.lon
    z[leader] = Drone[leader].location.global_relative_frame.alt
    distance_to_destination_initial = haversine_distance(x[leader], y[leader], waypoint.lat, waypoint.lon)
    print("Initial Distance:",distance_to_destination_initial)

    while(True):
        try:
            #Before each iteration, set velocities to zero
            x_dot, y_dot, z_dot = initialize_velocity()

            #Compute position of each drone
            x, y, z = compute_position(x, y, z)

            #Update the Adjacency Matrix
            A = update_adjacency_matrix(A, x, y)

            #Control law for the leader:
            #Compute position
            x[leader] = Drone[leader].location.global_relative_frame.lat
            y[leader] = Drone[leader].location.global_relative_frame.lon
            z[leader] = Drone[leader].location.global_relative_frame.alt
            #Compute velocity as per the control law. Z points downward
            x_goal = waypoint.lat
            y_goal = waypoint.lon
            z_goal = waypoint.alt
            heading = math.atan2(y_goal-y[leader], x_goal-x[leader])
            x_dot[leader] = nav_speed * math.cos(heading)
            y_dot[leader] = nav_speed * math.sin(heading)
            z_dot[leader] = -k4 * (z_goal - z[leader])

            #Control law for the followers:
            #Rigid formation in the X-Y Plane, Rendezvous about the Z-axis
            for i in range(N):
                if i != leader:
                    for j in range(N):
                        rigid_weight = k3 * (haversine_distance(x[i],y[i],x[j],y[j]) - W[i][j])
                        x_dot[i] -= A[i][j] * rigid_weight * k1 * (x[i] - x[j])
                        y_dot[i] -= A[i][j] * rigid_weight * k1 * (y[i] - y[j])
                        z_dot[i] -= -(A[i][j] * k2 * (z[i] - z[j]))

            #Send velocity command to each drone
            for i in range(N):
                send_global_ned_velocity(Drone[i], speed_bound(x_dot[i]), speed_bound(y_dot[i]), speed_bound(z_dot[i]))

            #Check if reached the waypoint
            if is_navigation_done(distance_to_destination_initial, waypoint, x[leader], y[leader]):
                break
            time.sleep(dt)

        except KeyboardInterrupt:
            break
    land()


#Mission:
connect_to_python()
for i in range(N):
    print("Arming Drone",i+1)
    arm(Drone[i])
    print("Takeoff initiated for Drone",i+1)
    takeoff(Drone[i], 1)
time.sleep(1)

rendezvous(A)
rigid_formation(A, W_line)
fly_in_formation(pickup_coords, A, W_line)
time.sleep(10)

for i in range(N):
    print("Arming Drone",i+1)
    arm(Drone[i])
    print("Takeoff initiated for Drone",i+1)
    takeoff(Drone[i], 1)
time.sleep(1)

rendezvous(A)
rigid_formation(A, W_square)
fly_in_formation(delivery_coords, A, W_square)
time.sleep(10)

for i in range(N):
    print("Arming Drone",i+1)
    arm(Drone[i])
    print("Takeoff initiated for Drone",i+1)
    takeoff(Drone[i], 1)

rigid_formation(A, W_triangle)
fly_in_formation(base_coords, A, W_triangle)
disarm()