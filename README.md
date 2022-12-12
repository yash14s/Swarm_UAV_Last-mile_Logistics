# Swarm_UAV_Last-mile_Logistics

A project by Abhinandan Krishnan, Navami Prabhu, Prerana Kolipaka, and Yash Srivastava
For the course ECE6563 - Networked Control Systems at Georgia Tech. Offered in Fall 2022, taught by Professor Chaouki T. Abdallah.

This project does Swarm UAV formation control for last-mile logistics applications. 

Flight stack: Ardupilot multi-copter
Simulator: Gazebo
Dronekit-Python for communicating with the Drone instance (virtual) over MAVLink. 

System Requirement:
Ubuntu 18.04 LTS machine with ROS Melodic, Ardupilot-Gazebo plugin installed. Refer https://github.com/yash14s/Drone/tree/main/installation%20and%20setup for setup instructions.

Usage:
i. In terminal(A): 
    $ roslaunch ncs delivery.launch
		Should see 4 drones in the gazebo world.
ii. In terminal(B):
		$ conda activate py3env
		$ sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0
		This will connect the ardupilot to the 1st drone (I0).
iii. In terminal(C): 
		$ conda activate py3env
		$ sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I1
		This will connect the ardupilot to the 2st drone (I1).
iv. In terminal(D): 
		$ conda activate py3env
		$ sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I2
		This will connect the ardupilot to the 3rd drone (I2).
v. In terminal(E): 
		$ conda activate py3env
		$ sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I3
		This will connect the ardupilot to the 4th drone (I4).
vi. In terminal(F):
    $ conda activate py3env
    $ python main.py
