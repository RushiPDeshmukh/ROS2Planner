# This code will run a pygame simulator 
# It will take the maze_array from the maze_generator and create a simulation of the maze
# It will check for the pos of the car and update it on the screen.

import rclpy 
from graph_interfaces.srv import GetGraph
from rclpy.node import Node
import pygame
import numpy as np

#parameters 
magnification_factor = 20
WHITE = (250,250,250)
BLACK = (0,0,0)

class MazeFetcher(Node):
    def __init__(self):
        super().__init__("MazeFetcher")
        self.client = self.create_client(GetGraph,"getMaze")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetGraph.Request()                                

    def send_request(self):
        self.future = self.client.call_async(self.req)

  

class grid_node:
    def __init__(self,loc):
        self.loc = loc
        self.connected_nodes = []
        self.possible_moves = []
        self.node_width = magnification_factor
        self.x = loc[0]*self.node_width
        self.y = loc[1]*self.node_width

    def add_connection(self,connected_node_loc,heading_direction):
        self.connected_nodes.append(connected_node_loc)
        self.possible_moves.append(heading_direction)

    def draw(self,win):
        if "N" not in self.possible_moves:
            pygame.draw.line(win,BLACK,(self.x,self.y),(self.x+self.node_width,self.y),3)
        if "S" not in self.possible_moves:
            pygame.draw.line(win,BLACK,(self.x,self.y+self.node_width),(self.x+self.node_width,self.y+self.node_width),3)
        if "E" not in self.possible_moves:
            pygame.draw.line(win,BLACK,(self.x+self.node_width,self.y),(self.x+self.node_width,self.y+self.node_width),3)
        if "W" not in self.possible_moves:
            pygame.draw.line(win,BLACK,(self.x,self.y),(self.x,self.y+self.node_width),3)

class Simulator_Node(Node):
    def __init__(self,adj_matrix,nodes,cols,rows,win):
        super().__init__("Simulator_Node")
        self.counter=0
        self.adj_matrix = adj_matrix
        self.win = win
        self.nodes = nodes
        self.cols = cols
        self.rows = rows
        self.createMaze()
        
        self.get_logger().info("Welcome to the simulator node:")
        self.create_timer(1.0,self.timer_callback)

    def timer_callback(self):
        self.counter +=1
        self.get_logger().info("Simulation Time:%s"+str(self.counter))
        self.draw()
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()

    def createMaze(self):
        cols = self.cols
        self.Maze = np.empty((self.cols,self.rows),dtype=grid_node)
        for i in range(self.nodes):
            curr_node = (i%cols,i//cols)
            self.Maze[curr_node[0]][curr_node[1]] = grid_node(curr_node)
            for j in range(self.nodes):
                if(self.adj_matrix[i][j]==1):
                    next_node = (j%cols,j//cols)
                    if next_node[1]==curr_node[1]-1:
                        dir = "N"
                    elif next_node[1]==curr_node[1]+1:
                        dir = "S"
                    elif next_node[0]==curr_node[0]+1:
                        dir = "E"  
                    elif next_node[0]==curr_node[0]-1:
                        dir = "W"   
                    self.Maze[curr_node[0]][curr_node[1]].add_connection(next_node,dir)
    
    def draw(self):
        self.win.fill(WHITE)
        for i in self.Maze:
            for j in i:
                j.draw(self.win)
        pygame.display.update()
                

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
                width = cols*magnification_factor
                height = rows*magnification_factor
                win = pygame.display.set_mode((width,height))
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
    
    

    sim1 = Simulator_Node(adj_matrix,nodes,cols,rows,win)
    rclpy.spin(sim1)

    rclpy.shutdown()


if __name__ == "__main__":
    main()