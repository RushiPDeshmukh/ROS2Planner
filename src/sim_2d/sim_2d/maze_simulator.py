# This code will run a pygame simulator 
# It will take the maze_array from the maze_generator and create a simulation of the maze
# It will check for the pos of the car and update it on the screen.

import rclpy 
from rclpy.node import Node

class Simulator_Node(Node):
    def __init__(self):
        super().__init__("Simulator_Node")
        self.get_logger().info("Welcome to the simulator node:")
        self.create_timer(1.0,self.timer_callback)
        self.counter=0

    def timer_callback(self):
        self.counter +=1
        self.get_logger().info(str(self.counter))


def main(args=None):
    rclpy.init(args=args)
    
    sim1 = Simulator_Node()
    rclpy.spin(sim1)
    # 

    rclpy.shutdown()


if __name__ == "__main__":
    main()