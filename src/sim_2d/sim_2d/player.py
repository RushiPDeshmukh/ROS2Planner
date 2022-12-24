import rclpy 
from rclpy.node import Node
from graph_interfaces.srv import GetGraph
from graph_interfaces.msg import PlayerPos
from graph_interfaces.srv import GetPath

import random
import numpy as np



class MazeFetcher(Node):
    def __init__(self):
        super().__init__("MazeFetcher")
        self.client = self.create_client(GetGraph,"getMaze")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetGraph.Request()                                

    def send_request(self):
        self.future = self.client.call_async(self.req)



class PlanFetcher(Node):
    def __init__(self):
        super().__init__("PlanFetcher")
        self.client_ = self.create_client(GetPath,"getPlan")
        self.get_logger().info('Node Created successfully\n')  
        self.get_logger().info('Waiting for request...\n') 
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        self.req = GetPath.Request()

    def send_request(self,start,goal,nodes,rows,cols,flatten_matrix):
        self.req.target.start = start
        self.req.target.goal = goal
        self.req.nodes = nodes
        self.req.cols = cols
        self.req.rows = rows
        self.req.data = flatten_matrix
        self.future = self.client_.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class grid_node:
    def __init__(self,loc):
        self.loc = loc
        self.connected_nodes = []
        self.possible_moves = []

    def add_connection(self,connected_node_loc,heading_direction):
        self.connected_nodes.append(connected_node_loc)
        self.possible_moves.append(heading_direction)

class Player(Node):
    def __init__(self,start,goal,nodes,path):
        super().__init__("Player")
        self.nodes = nodes
        self.start_pos = start
        self.goal_pos = goal
        self.path = path
        self.publisher_= self.create_publisher(PlayerPos,"player",10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PlayerPos()
        if self.i >= len(self.path):
            rclpy.shutdown()
        else:
            msg.start = self.path[self.i]
            self.i+=1
            msg.goal = self.goal_pos
            msg.name = "player"+str(self.start_pos)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing:' +str(msg.start))

def main(args=None):
    rclpy.init(args=args)
    
    maze_fetcher = MazeFetcher()
    maze_fetcher.send_request()

    while rclpy.ok():
        rclpy.spin_once(maze_fetcher)
        if maze_fetcher.future.done():
            try:
                response = maze_fetcher.future.result()
                nodes = response.nodes
                cols = response.cols
                rows = response.rows
                flatten_maze = response.data
                adj_matrix = np.zeros((nodes,nodes))
                for iter,value in enumerate(flatten_maze):
                    i = iter%nodes
                    j = iter//nodes
                    adj_matrix[i][j] = value
                maze_fetcher.get_logger().info('The maze has been succesfully fetched!')
            except Exception as e:
                maze_fetcher.get_logger().info(
                    'Service call failed %r' % (e,))
            break
    
    start = random.randint(0,nodes-1)
    goal = random.randint(0,nodes-1)

    planFetcher1 = PlanFetcher()
    response = planFetcher1.send_request(start,goal,nodes,rows,cols,flatten_maze)

    player1 = Player(start,goal,nodes,list(response.path))
    rclpy.spin(player1)

    rclpy.shutdown()


if __name__ == "__main__":
    main()