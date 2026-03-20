import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os
import subprocess
import time

class StopSimulation(Node):
    def __init__(self):
        super().__init__("stop_simulation")
        self.subscription = self.create_subscription(
            Bool,
            '/stop_simulation',
            self.stop_simulation_callback,
            10
        )
        
        # Add the exact names of the executables you want to aggressively eliminate:
        self.target_processes = [
            "save_uav_info_node",
            "fixed_wing_dynamics_node",
            "gz",
        ]
    
    def stop_simulation_callback(self, msg):
        if msg.data:
            self.get_logger().warn("Shutdown signal received! Eliminating all simulation processes...")
            
            # Hunt down and kill each target process
            for proc in self.target_processes:
                self.get_logger().info(f"Executing killall -9 on {proc}...")
                # We use DEVNULL to hide the terminal output if the process is already dead
                subprocess.call(
                    ["killall", "-9", proc], 
                    stderr=subprocess.DEVNULL, 
                    stdout=subprocess.DEVNULL
                )
            
            # Give the system a brief moment to clear the memory
            time.sleep(1.0)
            
            self.get_logger().info("All targeted processes eliminated. Exiting master node...")
            os._exit(0)  # Forcefully stop this python node

def main(args=None):
    rclpy.init(args=args)
    node = StopSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()