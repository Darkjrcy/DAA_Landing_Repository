import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rosgraph_msgs.msg import Clock


# Defineth class:
class WaitGazebo(Node):
    def __init__(self):
        super().__init__('wait_gazebo')

        # Define a variable to check when the simulation is ready:
        self.gz_ready = False

        # Define the quality of service:
        qos_profile_clock = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # SUbscrabe to the clock topic:
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile_clock)

    
    # Define the clock subscription callbakc:
    def clock_callback(self, msg: Clock):
        # Define the time published in Clock:
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        # If is more htan 15 (s) in sim change to ready to true and start the next commands:
        if t >= 2.0 and not self.gz_ready:
            self.gz_ready = True
            self.get_logger().info(f"Gazebo is ready! /clock={t:.3f}s. Shutting down...")
            rclpy.shutdown()


# Defain the main logic of the Node:
def main(args=None):
    rclpy.init(args=args)
    node = WaitGazebo()

    # Define the type of executor:
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Spin the executor:
    try:
        while rclpy.ok() and not node.gz_ready:
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()

    
if __name__ == '__main__':
    main()