from graph_interfaces.srv import GetGraph    # CHANGE

import rclpy
from rclpy.node import Node
import numpy as np
import random

#ROS2 Code 
class MazeGenerator(Node):

    def __init__(self,cols,rows):
        super().__init__('MazeGenerator')
        self.maze = Maze(cols,rows)
        self.cols = cols
        self.rows = rows
        self.srv = self.create_service(GetGraph, 'getMaze', self.get_graph_callback)     
        self.get_logger().info('Node Created successfully\n')  
        self.get_logger().info('Waiting for request...\n') 
    def get_graph_callback(self, request, response):
        self.maze.generate()
        response.nodes, matrix= self.maze.generate_adj_matrix()
        response.data = [int(i) for i in np.reshape(matrix,-1)]
        response.cols = self.cols
        response.rows = self.rows

        self.get_logger().info('Incoming request\n') # CHANGE

        return response

#Maze Generation Code

class grid_node:
    def __init__(self,loc):
        self.loc = loc
        self.connected_nodes = []
        self.possible_moves = []

    def add_connection(self,connected_node_loc,heading_direction):
        self.connected_nodes.append(connected_node_loc)
        self.possible_moves.append(heading_direction)
    
class Maze:
    def __init__(self,columns,rows) -> None:
        self.maze_nparray = np.empty((columns,rows),dtype=grid_node)
        self.cols = columns
        self.rows = rows
        for i in range(self.cols):
            for j in range(self.rows):
                self.maze_nparray[i][j] = grid_node((i,j))

    def generate(self):
        visited = []
        stack = []
        stack.append((0,0))

        while(len(visited)!=self.cols*self.rows):
            curr_node = stack[-1]
            if curr_node not in visited:
                visited.append(curr_node)
            heading_direction = random.choice(self._valid_directions(curr_node,visited))
            
            if heading_direction == "N":
                next_node = curr_node[0],curr_node[1]-1
                prev_heading_direction = "S"
                
            elif heading_direction == "S":
                next_node = curr_node[0],curr_node[1]+1
                prev_heading_direction = "N"
                
            elif heading_direction == "E":
                next_node = curr_node[0]+1,curr_node[1]
                prev_heading_direction = "W"

            elif heading_direction == "W":
                next_node = curr_node[0]-1,curr_node[1]
                prev_heading_direction = "E"

            if heading_direction !="X":
                stack.append(next_node)
                self.maze_nparray[curr_node[0]][curr_node[1]].add_connection(next_node,heading_direction)
                self.maze_nparray[next_node[0]][next_node[1]].add_connection(curr_node,prev_heading_direction)
            else:
                stack.pop()
            
    def generate_adj_matrix(self):
        nodes = self.cols*self.rows
        matrix = np.zeros((nodes,nodes),dtype=np.int64)
        for i in range(nodes):
            for j in range(nodes):
                curr_node = (i%self.cols,i//self.cols)
                next_node = (j%self.cols, j//self.cols)
                if next_node in self.maze_nparray[curr_node[0]][curr_node[1]].connected_nodes:
                    matrix[i][j] = 1
        return nodes,matrix

    def _valid_directions(self,curr_loc,visited):
        x,y = curr_loc
        valid_actions = []
        if(y-1 >= 0) and (x,y-1) not in visited:
            valid_actions.append("N")
        if(y+1 < self.rows) and (x,y+1) not in visited:
            valid_actions.append("S")
        if(x+1 < self.cols) and (x+1,y) not in visited:
            valid_actions.append("E")
        if(x-1 >= 0) and (x-1,y) not in visited:
            valid_actions.append("W")
        if not valid_actions:
            valid_actions.append("X")
        return valid_actions
    


def main(cols =20   ,rows = 12,args=None):
    rclpy.init(args=args)

    Generator = MazeGenerator(cols,rows)

    rclpy.spin(Generator)

    rclpy.shutdown()

if __name__ == '__main__':
    main()