1. Add iq_sim's latest gazebo models
	i. $ echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc

2. Create multiple instances of a drone.
	i. As shown in [2], use GUI to navigate to ~/catkin_ws/src/iq_sim
	ii. Create multiple copies of drone1, call them drone2, drone3,... dronen
	iii. For each drone_i, edit model.config and change the name to drone_i. eg: drone2 for drone2, drone3 for drone3, and so on..
	iv. Edit the model.sdf for each drone_i. Update the fdm port as shown in [1].

3. Add multiple drones to a gazebo world.
	i. Create a copy of iq_sim's runway.world and call it runway_swarm.world
	ii. Edit runway_swarm.world. Replace lines shown in [3] with those in [4]. 
	iii. Create a launch file for runway_swarm.world, call it runway_swarm.launch.

4. Launch the world and connect each drone to ardupilot
	i. In terminal(A): $ roslaunch iq_sim runway_swarm.launch
		Should see 3 drones in the gazebo world.
	ii. In terminal(B):
			$ conda activate py3env
			$ sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0
		This will connect the ardupilot to the 1st drone (I0).
	iii. In terminal(C): 
			$ conda activate py3env
			$ sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I1
		Note that we used I1 for connecting to the 2nd drone.
	iv. Similarly, repeat step (iii) for each of the remaining drones.
	Note: The first drone is available on TCP ports 5760, 5762, 5763. The 2nd drone is at 5770, 5772, 5773. Similarly, the next drone is at an increment of 10.
	v. Mission planner: Connect to the 1st drone as usual. For subsequent drones, right click on the top bar of mission planner, select connection options -> TCP set baud rate as 115200. Select port 5773 for drone2. Similarly for other drones.

5. Flight test using MAVProxy
	i. Each of the drones can be controlled individually by issuing MAVProxy commands from their respective terminals

6. Flight test using dronekit


Appendix:
[1] https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/swarming_ardupilot.md 
[2] https://youtu.be/r15Tc6e2K7Y
[3] <model name="iris">
      <include>
        <uri>model://drone_with_camera</uri>
      </include>
      <pose> 0 0 0 0 0 0</pose>
    </model>
[4] <model name="drone1">
      <pose> 0 0 0 0 0 0</pose>
      <include>
        <uri>model://drone1</uri>
      </include>
    </model>
    <model name="drone2">
      <pose> 2 0 0 0 1.57</pose>
      <include>
        <uri>model://drone2</uri>
      </include>
    </model>
    <model name="drone3">
      <pose> 4 0 0 0 0 3.14</pose>
      <include>
        <uri>model://drone3</uri>
      </include>
    </model>
	
