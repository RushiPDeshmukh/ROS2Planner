import rclpy
import numpy as np
from rclpy.node import Node
from graph_interfaces.srv import GetPath


class Planner(Node):
    def __init__(self):
        super().__init__("Planner")
        self.server_ = self.create_service(GetPath,"getPlan",self.service_callback)
        self.get_logger().info('Node Created successfully\n')  
        self.get_logger().info('Waiting for request...\n') 


    def service_callback(self,request,response):
        self.get_logger().info('Incoming request\n') # CHANGE
        self.start = request.target.start
        self.goal = request.target.goal
        if self.start == self.goal:
            response.path = [self.start]
            return response
        self.nodes = request.nodes
        self.rows = request.rows
        self.cols = request.cols
        flatten_maze = request.data
        adj_matrix = np.zeros((self.nodes,self.nodes))
        for iter,value in enumerate(flatten_maze):
            i = iter%self.nodes
            j = iter//self.nodes
            adj_matrix[i][j] = value
        self.convert_adjList(adj_matrix)
        response.path = self.bfs_planner()
        return response

    def convert_adjList(self,adj_matrix):
        self.adjList = {}
        for i in range(len(adj_matrix)):
            self.adjList[i] = []
            for j in range(len(adj_matrix)):
                if adj_matrix[i][j] == 1:
                    self.adjList[i].append(j)

        
    def bfs_planner(self):
        visited = np.ones(self.nodes)*-1
        queue = []
        path = []
        parent = np.ones(self.nodes,dtype=np.int64)*0
        queue.append(self.start)
        self.get_logger().info('Start Node is \n'+str(self.start))
        self.get_logger().info('Goal Node is \n'+str(self.goal))
        while queue:
            curr_node = queue.pop(0)
            visited[curr_node] = 1
            if curr_node == self.goal:
                break
            else:
                for next_node in self.adjList[curr_node]:
                    if visited[next_node] == -1:
                        queue.append(next_node)
                        parent[next_node] = curr_node
        temp_node = self.goal
        self.get_logger().info('Reverse path: \n')
        while temp_node!=self.start:
            path.append(int(temp_node))
            self.get_logger().info(str(temp_node))
            temp_node = parent[temp_node]
            
        
        path.reverse()
        return path


def main(cols =20   ,rows = 12,args=None):
    rclpy.init(args=args)

    bfs_planner = Planner()

    rclpy.spin(bfs_planner)

    rclpy.shutdown()

if __name__ == '__main__':
    main()