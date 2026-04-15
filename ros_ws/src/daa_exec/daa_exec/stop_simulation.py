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
    
        self.target_processes = [
            "ruby",      
            "gz-sim",
            "gz sim",
            "ros2",
            "--ros-args"
        ]
    
    def stop_simulation_callback(self, msg):
        if msg.data:
            self.get_logger().warn("Shutdown signal received! Eliminating all simulation processes...")
            
            for proc in self.target_processes:
                self.get_logger().info(f"Executing pkill -9 -f on {proc}...")
                subprocess.call(
                    ["pkill", "-9", "-f", "--", proc], 
                    stderr=subprocess.DEVNULL, 
                    stdout=subprocess.DEVNULL
                )
            
            # Give the system a brief moment to clear the memory
            time.sleep(1.0)
            
            self.get_logger().info("All targeted processes eliminated. Exiting master node...")
            os._exit(0) 

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