from cProfile import label
from enum import Flag
import os
import math
from queue import Empty
from tkinter import PROJECTING, S
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import itertools

# Library to change the orientation angles to quaternion:
from scipy.spatial.transform import Rotation as R

# ROS2 Packages:
import rclpy
from rclpy.node import Node
# Libary to define hte QUality of service of ROS2 messages:
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# Messages required:
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from gnss_multipath_plugin.msg import AdsbInfo
from uav_dynamics.msg import AvoidanceStates
from nav_msgs.msg import Odometry

# Libraries to start other executables on the command window:
import subprocess
import os 
import contextlib
import signal
import atexit



# Function to pass the waypoints from a string to ana array:
def parse_wp_string(wp_string):
    # To make it work the Waypoints would need to have the next format "North, East, Down, Overall Speed" in feets at each waypoint:"
    rows =[]
    for wp in wp_string.split(';'):
        wp = wp.strip()
        if not wp:
            continue
        nums = [float(x) for x in wp.split(',')]
        rows.append(nums)
    parsed = np.array(rows, dtype=float)
    
    # Now save the waypoints so each element is in a column
    return np.column_stack([
        parsed[:,0] * 0.3048,   # N (m)
        parsed[:,1] * 0.3048,   # E (m)
        parsed[:,2] * -0.3048,  # Altitude (m)
        parsed[:,3] * 0.3048,   # Speed (m/s)
    ])



# Parse multiple waypoint strings from the overall string:
def parse_multiple_wp_strings(mult_wp_string):
    # This would split the different waypoint strings divided by % in the overall string of the UAVs
    # and save the inforamtion in numerical arrays lists to eb inputed in the dictionaries
    segments = []
    for seg in mult_wp_string.split('%'):
        seg = seg.strip()
        if not seg:
            continue
        segments.append(parse_wp_string(seg))
    return segments

# Format string from numbers:
def _format_num(x: float) -> str:
    # neat, short float formatting (no trailing zeros)
    s = f"{x:.6f}".rstrip('0').rstrip('.')
    return s if s else "0"



# Function to separate the multiple waypoints in a list of strings:
def separate_wp(mult_wp_string):
    waypoints_strings = []
    for wps in mult_wp_string.split('%'):
        wps = wps.strip()
        # Pass from ft to meters:
        out_wps = []
        for wp in wps.split(';'):
            wp = wp.strip()
            parts = [p.strip() for p in wp.split(',')]
            N_ft, E_ft, D_ft, V_ftps = map(float, parts)
            # Update it to meters:
            N_m = N_ft * 0.3048
            E_m = E_ft * 0.3048
            Down_m = D_ft * 0.3048
            Vel_mps = V_ftps * 0.3048

            # Save in out_waypooints as a new list:
            out_wps.append(f"{_format_num(N_m)}, {_format_num(E_m)}, {_format_num(Down_m)}, {_format_num(Vel_mps)}")
        
        # Save the modified waypoint strings:
        waypoints_strings.append('; '.join(out_wps))
    return waypoints_strings



# Generate the color-markers for the intruders:
def generate_airplanes_color_maps(names):
    palette = plt.cm.get_cmap('tab20').colors
    ls_cycle = ['--','-',':']
    color_map = {}
    count = 0
    for name in names:
        color = palette[count % len(palette)]
        line_stl = ls_cycle[count % len(ls_cycle)]
        color_map[name] = {"color": color, "line_style" : line_stl}
        count += 1
    return color_map



# Start the executable:
class DAASimulation(Node):
    def __init__(self):
        # Generate the Node:
        super().__init__('daa_simulation')

        # Declare the directory where the information is going to be saved:\
        self.declare_parameter('data_directory', '/home/adcl/AirplanePathFollower/DATA/DAA_Fligths') # Folder to save the inforamtion
        self.save_dir = self.get_parameter('data_directory').value

        # Declare the world name:
        self.declare_parameter('world_name')
        self.world_name = self.get_parameter('world_name').value

        # Define the plot characteristics:
        self.model_lines = {}

        # Boolean to say when teh mission started:
        self.mission_started = False

        # Define the Models and if te avoidance type:
        self.declare_parameter('model_names', ['airplane_1', 'airpalne_2'])
        self.declare_parameter('uav_type', ['fixed_wing', 'fixed_wing'])
        self.declare_parameter('model_waypoints', [
            '0,50,-50,5;450,85,-20,5 % 500,100,-20,6', 
            '10,60,-50,5;460,95,-20,5 % 510,110,-20,6'  
        ])
        self.declare_parameter('has_avoidance', [True, False])
        self.declare_parameter('avoidance_types', ['GEOMETRIC', 'NO'])
        # Save the values:
        self.model_names = self.get_parameter('model_names').value
        uav_type = self.declare_parameter('uav_type').value
        model_wp_raw = self.get_parameter('model_waypoints').value
        has_avoidance = self.get_parameter('has_avoidance').value
        avoidance_types = self.get_parameter('avoidance_types').value

        # Define vectors to save the avoiders and the intruders:
        # AVoiders:
        self.avoiders_info_string = {}
        self.avoiders_info = {}
        self.avoiders_type = {}
        # Intruders
        self.intruders_info_string = {}
        self.intruders_info = {}
        # UAV type:
        self.uav_type = {}
        # Forloop between the model_names:
        for i in range(len(self.model_names)):
            if has_avoidance[i]:
                self.avoiders_info_string[self.model_names[i]] = separate_wp(model_wp_raw[i])
                self.avoiders_info[self.model_names[i]] = parse_multiple_wp_strings(model_wp_raw[i])
                self.avoiders_type[self.model_names[i]] = avoidance_types[i]
            else:
                self.intruders_info_string[self.model_names[i]] = separate_wp(model_wp_raw[i])
                self.intruders_info[self.model_names[i]] = parse_multiple_wp_strings(model_wp_raw[i])
            self.uav_type[self.model_names[i]] = uav_type[i]
        

        # Start the plot for the airpalne Trahectory:
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.show(block=False)
        # Generate the colors for the aircrafts:
        self.color_map = generate_airplanes_color_maps(self.model_names)

        # QoS required for the transmision of the ADS-B and the states:
        q_reliable = QoSProfile(depth=10)
        q_reliable.reliability = ReliabilityPolicy.RELIABLE
        q_reliable.durability  = DurabilityPolicy.VOLATILE
        # QoS for the trajectory complete flag topic of all the intruders
        q_events = q_reliable

        # Create a subscriber to the traj_complete of any of hte uavs:
        self.trajectory_complete_sub_ = self.create_subscription(Bool, '/traj_complete', self.traj_complete_callback, q_events)

        # Vector to save the running processess of movement and save information:
        self.running_procs = {}
        self.save_info_procs = {}

        # Create subscribers to the states and ads-b of each model:
        self.models_states_subs = []
        self.models_adsb_subs = []
        # Create dictionaries to save the real position of the models to be plotted:
        self.models_states_north_ft = {}
        self.models_states_east_ft = {}
        self.models_states_up_ft = {}
        # Create dictionaries to save all the infroamtion posted on the adsb-b of the models:
        self.models_adsb_north = {}
        self.models_adsb_east = {}
        self.models_adsb_up = {}
        self.models_adsb_v_north = {}
        self.models_adsb_v_east = {}
        self.models_adsb_v_up = {}
        self.models_adsb_course = {}
        self.models_adsb_fpa = {}
        self.models_adsb_roll = {}
        self.models_adsb_p = {}
        self.models_adsb_q = {}
        self.models_adsb_r = {}
        # Forloop in all the models:
        for i in range(len(self.model_names)):
            # Subscribe to the states to generate the plots of the trajectories:
            state_sub = self.create_subscription(
                Odometry, f"/{self.model_names[i]}/states", 
                lambda msg, n=self.model_names[i]: self.states_callback(msg,n),
                q_reliable
            )
            self.models_states_subs.append(state_sub)
            # Create vectors to save the position that later is going to be plot:
            self.models_states_north_ft[self.model_names[i]] = []
            self.models_states_east_ft[self.model_names[i]] = []
            self.models_states_up_ft[self.model_names[i]] = []

            # Subscribe to the adsb information to publish to the DAA algorithms:
            adsb_sub = self.create_subscription(
                AdsbInfo, f"/{self.model_names[i]}/adsb_info",
                lambda msg, n=self.model_names[i]: self.adsb_callback(msg,n),
                q_reliable
            )
            self.models_adsb_subs.append(adsb_sub)
            # Create lits to save the states give by hte ADS-B:
            self.models_adsb_north[self.model_names[i]] = []
            self.models_adsb_east[self.model_names[i]] = []
            self.models_adsb_up[self.model_names[i]] = []
            self.models_adsb_v_north[self.model_names[i]] = []
            self.models_adsb_v_east[self.model_names[i]] = []
            self.models_adsb_v_up[self.model_names[i]] = []
            self.models_adsb_course[self.model_names[i]] = []
            self.models_adsb_fpa[self.model_names[i]] = []
            self.models_adsb_roll[self.model_names[i]] = []
            self.models_adsb_p[self.model_names[i]] = []
            self.models_adsb_q[self.model_names[i]] = []
            self.models_adsb_r[self.model_names[i]] = []

        
        # Create a publisher to the start_mission topic:
        self.start_mission_pub = self.create_publisher(Bool, "/mission_starts", 1)

        # QoS for the publishers
        qos_2 = QoSProfile(depth=1)
        qos_2.reliability = ReliabilityPolicy.RELIABLE
        qos_2.durability = DurabilityPolicy.VOLATILE
        # Create a publisher of hte ADS-B intruders information and a timer for every avoider:
        self.adsb_publishers = {}
        for avoider_name in self.avoiders_type.keys():
            self.adsb_publishers[avoider_name] = self.create_publisher(AvoidanceStates, 
                f"/{avoider_name}/obstacles_adsb", qos_2)
        # Create a timer to publisht he adsb intruders information:
        self.adsb_timer  = self.create_timer(1/4, self.adsb_pub_callback)

        # Restart the variables before starting the siomulation, or every time a new trajecotry is started:
        self.restart_varibles()

        # Publisher to stop the simulation completely:
        self.stop_pub = self.create_publisher(Bool,'/stop_simulation',1)

        # Process to cloase all the ndoes when is manually killed:
        atexit.register(self._cleanup, reason = "atexit")
        try:
            rclpy.get_default_context().on_shutdown(lambda: self._cleanup("rclpy.on_shutdown"))
        except Exception:
            pass
        # Tell the type of close:
        signal.signal(signal.SIGINT,  self._sig_handler)  
        signal.signal(signal.SIGTERM, self._sig_handler)




    # Function to tell all topic listeners to stop, when the simualiton is stopped:
    def _sig_handler(self, signum, frame):
        self.get_logger().warn(f"Signal {signum} received -> stopping followers")
        with contextlib.suppress(Exception):
            self.stop_pub.publish(Bool(data=True))
        self.stop_wait_the_followers(grace1=1.5, grace2=1.5)


    
    # Clean-up funciton to clean all teh nodes when is being manualed killed:
    def _cleanup(self, reason = ""):
        try:
            self.get_logger().info(f"Cleanup invoked ({reason})")
        except Exception:
            pass
        with contextlib.suppress(Exception):
            self.stop_pub.publish(Bool(data=True))
        
        # Try it first easily and later harder:
        self.stop_wait_the_followers(grace1=1.5, grace2=1.5)
        with contextlib.suppress(Exception):
            import matplotlib.pyplot as plt
            plt.close('all')
    


    # Function to setup the visualization plot:
    def setup_plot(self, num_encounter):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Call the axis fo the plot
        self.ax.clear()
        self.ax.set_xlabel('East (ft)')
        self.ax.set_ylabel('North (ft)')
        self.ax.set_zlabel('Altitude (ft)')
        self.ax.set_title('UAV Waypoint follower Monitoring')
        # Gnerate three lists to save the limits of the graph:
        east_points, north_points, alt_points = [], [], []

        # Plot the Avoiders trajectories:
        for name, segments in self.avoiders_info.items():
            # Open the respective trajecotry:
            avoid_traject = segments[num_encounter]
            avoid_e_ft = avoid_traject[: , 1] / 0.3048
            avoid_n_ft = avoid_traject[: , 0] / 0.3048
            avoid_u_ft = avoid_traject[: , 2] / 0.3048
            # Save them in the lit to create the limits:
            east_points.append(avoid_e_ft)
            north_points.append(avoid_n_ft)
            alt_points.append(avoid_u_ft)
            # PLot the AVoider Trajctory:
            self.ax.plot(avoid_e_ft,avoid_n_ft,avoid_u_ft,
                        color = self.color_map[name]['color'],
                        linestyle=self.color_map[name]['line_style'],
                        alpha = 0.9,
                        label=f"AVoider {name}")

        # PLot the Intruders trajecotories:
        for name, segments in self.intruders_info.items():
            # Open the respective trajctory:
            intr_traejct = segments[name]
            intr_e_ft = intr_traejct[: , 1] / 0.3048
            intr_n_ft = intr_traejct[: , 0] / 0.3048
            intr_u_ft = intr_traejct[: , 2] / 0.3048
            # Save them in the lit to create the limits:
            east_points.append(intr_e_ft)
            north_points.append(intr_n_ft)
            alt_points.append(intr_u_ft)
            # PLot the intruder traejctory:
            self.ax.plot(intr_e_ft,intr_n_ft,intr_u_ft,
                        color = self.color_map[name]['color'],
                        linestyle=self.color_map[name]['line_style'],
                        alpha = 0.9,
                        label=f"Intruder {name}")
        
        # Generte the dynamics plots for the models:
        for name in self.model_names:
            (line,) = self.ax.plot([],[],[],'m')
            self.model_lines[name] = line
        
        # Plot the legeneds:
        self.ax.legend()

        # Find teh limits of the Waypoints to do an axis equal:
        e_all = np.concatenate(east_points)
        n_all = np.concatenate(north_points)
        alt_all = np.concatenate(alt_points)
        max_range = max( np.ptp(e_all),np.ptp(n_all),np.ptp(alt_all)) / 2.0
        mid_x = np.mean(e_all)
        mid_y = np.mean(n_all)
        mid_z = np.mean(alt_all)
        self.ax.set_xlim(mid_x - max_range, mid_x + max_range)
        self.ax.set_ylim(mid_y - max_range, mid_y + max_range)
        self.ax.set_zlim(mid_z - max_range, mid_z + max_range)
        plt.show(block=False)
            




    # FUnction to restart the varaibles:
    def restart_varibles(self):
        # Flag to see if any UAV compelte the trejctory:
        self.traj_complete = False
        # Information savers for the plottong and adsb data:
        hist_models =[
            # Position for plotin:
            self.models_states_east_ft,
            self.models_states_north_ft,
            self.models_states_up_ft,
            # ADS-B information:
            self.models_adsb_north,
            self.models_adsb_east,
            self.models_adsb_up,
            self.models_adsb_v_east,
            self.models_adsb_v_north,
            self.models_adsb_v_up,
            self.models_adsb_course,
            self.models_adsb_fpa,
            self.models_adsb_roll
        ]

        # Erase the running process of ROS2 that need to be closed:
        self.running_procs = {}

        # Clear all the values inside the hist_models_info:
        for list_hist in hist_models:
            for k in list_hist:
                list_hist[k].clear()






    # Detect when the Airplane its near the goal position:
    def traj_complete_callback(self,msg, name):
        if msg.data and not getattr(self, "traj_complete", False):
            self.traj_complete = True
            self.get_logger().info(f"{name} completed the trajectory; signaling encounter completion")
    


    # Model States callback to create the plot:
    def states_callback(self, msg, model_name):
        # If this intruder doesn't have a line, ignore
        if model_name not in self.model_lines:
            return        

        # Analyze when all the intruders start to move 
        if self.mission_started:
            # Get the position of hte models:
            east_ft = msg.pose.pose.position.x / 0.3048
            north_ft = msg.pose.pose.position.y / 0.3048
            alt_ft = msg.pose.pose.position.z / 0.3048
            # Save the position in dictironary lists:
            self.models_states_east_ft[model_name].append(east_ft)
            self.models_states_north_ft[model_name].append(north_ft)
            self.models_states_up_ft[model_name].append(alt_ft)

            # Plot the trajectory:
            line_of_model = self.model_lines[model_name]
            line_of_model.set_data(self.models_states_east_ft[model_name], self.models_states_north_ft[model_name])
            line_of_model.set_3d_properties(self.models_states_up_ft[model_name])
            plt.draw()
            plt.pause(0.001)
    


    # ADS-B models callback to save the information and later publish it:
    def adsb_callback(self, msg, model_name):
        # If this intruder doesn't have a line, ignore
        if model_name not in self.model_lines:
            return        
        
        # Start saving when the mission stated:
        if self.mission_started:
            # Save the ads-b data in vectors:
            self.models_adsb_north[model_name].append(msg.north)
            self.models_adsb_east[model_name].append(msg.east)
            self.models_adsb_up[model_name].append(msg.up)
            self.models_adsb_v_north[model_name].append(msg.v_north)
            self.models_adsb_v_east[model_name].append(msg.v_east)
            self.models_adsb_v_up[model_name].append(msg.v_up)
            self.models_adsb_course[model_name].append(msg.course)
            self.models_adsb_fpa[model_name].append(msg.fpa)
            self.models_adsb_roll[model_name].append(msg.roll)
            self.models_adsb_p[model_name].append(msg.p)
            self.models_adsb_q[model_name].append(msg.q)
            self.models_adsb_r[model_name].append(msg.r)



    # Function to spawn the model using the gz service:
    def call_service_change_position(self, entity_name: str, east, north, alt, roll, fpa, course) -> bool:
        # Convert euler angles to quaternions:
        yaw = np.pi/2 - course
        pitch = -fpa
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        # Build the gazebo command:
        gz_str = (
            f"name: '{entity_name}', "
            f"position: {{x: {east}, y: {north}, z: {alt}}}, "
            f"orientation: {{x: {quat[0]}, y: {quat[1]}, z: {quat[2]}, w: {quat[3]}}}"
        )
        # Constructed and send it to the command window:
        cmd = [
            "gz", "service", "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", gz_str
        ]

        # Try to run the command and wait until it finish to send a boolean:
        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.get_logger().info(f"Successfully moved {entity_name} via GZ direct service.")
            return True
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to move {entity_name}. Error: {e.stderr}")
            return False
    


    # Create teh timer function that publishes 
    def adsb_pub_callback(self):
        # Check if the mission started:
        if not self.mission_started:
            return
        
        # Loop in allt he voiders and the publishers:
        for avoider_name, publisher in self.adsb_publishers.items():
            # Create the aoidance states msg:
            msg = AvoidanceStates()

            # Check for all the other models taht are not the actual model 
            for model_name_i in self.model_names:
                # Cehk if the model_name is the avoider:
                if model_name_i == avoider_name:
                    continue
                # Additionally check if the information is not saved:
                if not self.models_adsb_north[model_name_i]:
                    continue

                # Add ADS-B messages to the Avoidance publisher:
                intr_states = AdsbInfo
                # Position:
                intr_states.north = self.models_adsb_north[model_name_i][-1]
                intr_states.east = self.models_adsb_east[model_name_i][-1]
                intr_states.up = self.models_adsb_up[model_name_i][-1]
                # Velocity:
                intr_states.v_north = self.models_adsb_v_north[model_name_i][-1]
                intr_states.v_east = self.models_adsb_v_east[model_name_i][-1]
                intr_states.v_up = self.models_adsb_v_up[model_name_i][-1]
                # Orientaion:
                intr_states.roll = self.models_adsb_roll[model_name_i][-1]
                intr_states.fpa  = self.models_adsb_fpa[model_name_i][-1]
                intr_states.course = self.models_adsb_course[model_name_i][-1]
                # Angular velocity:
                intr_states.p = self.models_adsb_p[model_name_i][-1]
                intr_states.q = self.models_adsb_q[model_name_i][-1]
                intr_states.r = self.models_adsb_r[model_name_i][-1]
            
                # ADD HTE ID and append the adsb info of the intruder:
                msg.intruder_states.append(intr_states)
                msg.obstacles_id.append(model_name_i)
            
            # NOw publish the message
            publisher.publish(msg)
    


    
    # Keep the system running while the followers are closing:
    def stop_wait_the_followers(self, grace1: float = 6.0, grace2: float = 5.0):
        if not self.running_procs:
            return
        
        # Give them time to close by themselves:
        t_end = time.time() + grace1
        while time.time() < t_end:
            if all(p.poll() is not None for p in self.running_procs.values()):
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop any remainning process that is still open;
        still_alive = [p for p in self.running_procs.values() if p.poll() is None]
        for p in still_alive:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                p.terminate()
            
        # Wait a bit more of time
        t_end = time.time() + grace2
        while time.time() < t_end:
            if all(p.poll() is not None for p in self.running_procs.values()):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Hard kill the stragglers:
        for name, p in list(self.running_procs.items()):
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:
                    p.kill()
            self.running_procs.pop(name, None)
    


    # Start the movement of the uavs:
    def start_uav_dynamics(self, idx):
        # For every uav launch the model for avoiders:
        for name, seg_list in self.avoiders_info_string.items():
            own_args = [
                "ros2", "run", "uav_dynamics", f"{self.uav_type[name]}_dynamics_node", name, seg_list[idx], self.avoiders_type[name], "1" 
            ]
            # Add it to the running process:
            self.running_procs[name] = subprocess.Popen(own_args, start_new_session=True)
        # Now fo it for every intruder:
        for name, seg_list in self.intruders_info_string.items():
            intr_args = [
                "ros2", "run", "uav_dynamics", f"{self.uav_type[name]}_dynamics_node", name, seg_list[idx], "NONE", "0" 
            ]
            # Add it to the running process:
            self.running_procs[name] = subprocess.Popen(intr_args, start_new_session=True)
    
    

    # Start the mission only once:
    def start_the_mission(self):
        # Check if hte mission has already started from anoehr funciton:
        if not self.mission_started:
            # Now started yourself:
            self.mission_started = True
            # Punlish it so all the other nodes see the message:
            msg = Bool()
            msg.data = True
            self.start_mission_pub.publish(msg)



    # Function to spawn the airplane at the initial position by using the first waypoints:
    def spawn_the_UAV_using_waypoints(self, name, trajectory):
        # Define the teo points:
        pos0 = trajectory[0,:]
        pos1 = trajectory[1,:]
        # Calcualte the required deltas:
        delta_pos = pos1 - pos0

        # Define the angles of orientation of the UAV:
        # Fligth path angle:
        fpa = np.arctan2(delta_pos[2], np.hypot(delta_pos[0], delta_pos[1]))
        # Course angle:
        course = np.arctan2(delta_pos[1], delta_pos[0]) 
        
        # Start the spawning:
        if not self.call_service_change_position(name, pos0[1], pos0[0], pos0[2], 0.0, fpa, course):
                        return
        time.sleep(5)

    

    # Start the saving nodes:
    def start_saving_nodes(self, saving_directory):
        for model in self.model_names:
            save_args = [
                "ros2", "run", "uav_bringup", "save_uav_info_node", model, saving_directory
            ]
            # Add it to the saving running process:
            self.save_info_procs[model] = subprocess.Popen(save_args, start_new_session=True)

    


    # Define the main function of the DAA simulation:
    def daa_simulation_setup(self):
        # Start the saving nodes:
        self.start_saving_nodes()
        # Detect the number of encounters:
        first_model = self.model_names[0]
        if first_model in self.avoiders_info:
            encounters_list = self.avoiders_info[first_model]
        else:
            encounters_list = self.intruders_info[first_model]            
        num_encounters = len(encounters_list)

        # Iterate thorugh the smiulation encounters:
        for i in range(num_encounters):
            self.get_logger().info(f"========== STARTING ENCOUNTER {i} ==========")
            # Restart the varaibles and wait some time before starting:
            self.restart_varibles()
            time.sleep(5)

            # Start the plot to see how the airpalnes are moving:







