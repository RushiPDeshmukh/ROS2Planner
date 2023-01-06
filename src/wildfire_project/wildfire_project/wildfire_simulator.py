""" This python code generates an environment with random obstacles.
    This code contains a class for bush with default state == 'bush' and can be changed to 'fire' state.
        - location of obstacle in environment.
        - state of obstacle
        - funtions to change the state
        - funtion to draw the obstacle
    
    This code contains a environment class forest 
        - list of obstacles
        - function to randomly change the state of obstacle based on time element
        - function to randomly generate a new environment
        -
    This code contains a quad_tree graph function.
    This code contains a PRM based roadmap generator function.
"""
import numpy as np
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from graph_interfaces.msg import Coordinates

#TO-DO
# 1. define a random patch() function this function defines a obstacle shape as numpy array -------------------------------------DONE
# 2. Define a rclpy node to subscribe to the topic "fire_truck_pos" -------------------------------------------------------------DONE
# 3. Define a rclpy Server node to send the the obstacle field information to the player in a 1D array as a flatten adjMatrix
# 4. Define a rclpy node to publish the list of fires in the simulated forest on the topic "forest_fire_pos" --------------------DONE

BLACK = (0,0,0)
WHITE = (255,255,255)

#Task 1 : Generate the obstacle field
class Bush:
    def __init__(self,pos, tree_width = 10) -> None:
        self.pos = pos #stores the topLeft corner pos of the forest patch
        self.x = pos[0]
        self.y = pos[1]
        self.tree_width = tree_width #this variable sets the width of single unit of the patch.
        self.shape = self.random_patch() #this function returns a random forest shape and its shape variable
        self.on_fire = False

    def get_data(self):
        return self.pos,self.shape

    def change_shape(self):
        self.shape = self.random_patch()
        return self.shape

    def set_fire(self):
        self.on_fire = True
        
    def extinguish_fire(self):
        self.on_fire = False

    def get_rect(self):
        return pygame.Rect(self.x,self.y,self.shape[0]*self.tree_width,self.shape[1]*self.tree_width)

    def get_boundary(self):
        x,y = self.pos
        center = ((self.shape[0]*self.tree_width + x)//2,(self.shape[1]*self.tree_width + y)//2)
        radius = ((center[0]-x)**2 + (center[1]+y)**2)**0.5
        return (center,radius)

    def draw(self,surface):
        x,y = self.pos
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):
                color = ((np.random.rand()*10+10)*(1+self.on_fire*10),(np.random.rand()*10+5)*(1+(not self.on_fire)*10),0)
                rect = (x+i*self.tree_width,y+j*self.tree_width,self.tree_width,self.tree_width)
                pygame.draw.rect(surface,color,rect,0,3)
    
    def random_patch(self):
        x = np.random.randint(0,4)
        y = np.random.randint(0,4)
        return (x,y)


class Forest_Node():
    def __init__(self,coverage = 10,width = 500,bush_width = 10) -> None:
        self.coverage = coverage
        self.width = width
        self.bush_width = bush_width
        self.number_of_bushes = 0
        self.bushes = []
        self.win = pygame.display.set_mode((1000,700))
        self.surf1 = pygame.surface.Surface((500,500))
        self.current_fires = []
        self.create()
        self.extinguish_boundary_radius = 30
        self.grid_data = np.zeros((self.width//(self.bush_width),self.width//(self.bush_width)))
    
    def extinguish_fire(self,car_pos):
        for bush in self.bushes:
            bush_center = bush.get_rect().center
            distance = ((car_pos[0]-bush_center[0])**2 + (car_pos[1]-bush_center[1])**2)**0.5
            if distance<self.extinguish_boundary_radius:
                bush.extinguish_fire()

    def create(self):
        """This function will randomly generate the Bush objects to populate the forest
        """
        self.bushes = []
        self.number_of_bushes = 0
        occupied = 0
        total_area = (self.width**2) #total area of forest
        coverage = occupied*100//total_area #current coverage of forest
        while coverage<self.coverage:
            #random pos in the forest
            random_x = np.random.randint(0,self.width//self.bush_width)*self.bush_width
            random_y = np.random.randint(0,self.width//self.bush_width)*self.bush_width

            #generate the bush object at this location
            bush = Bush((random_x,random_y),self.bush_width)
            _,shape = bush.get_data()
            counter = 3
            while not ((random_x+shape[0]*self.bush_width < self.width) and (random_y+shape[1]*self.bush_width < self.width)):
                shape = bush.change_shape()
                counter -=1
                if counter <0:
                    continue

            rect = bush.get_rect()
            #check if the bush fits inside the forest and add to the list of bushes 
            if not self.is_collision(rect):
                self.bushes.append(bush)
                self.number_of_bushes +=1
                occupied += shape[0]*shape[1]*self.bush_width**2
                for i in range(shape[0]):
                    for j in range(shape[1]):
                        self.grid_data[random_x+i][random_y+j]=1
            coverage = occupied*100//total_area

    def is_collision(self,rect2):
        for bush in self.bushes:
            rect1 = bush.get_rect()
            if rect1.colliderect(rect2):
                return True
        return False

    def get_grid(self):
        return self.grid_data
    
    def trigger_fires(self):
        """This function will randomly choose a bush to set on fire.
           This function will choose a bush only if it is not already on fire.
        """
        random_bush = np.random.randint(0,len(self.bushes))
        self.bushes[random_bush].set_fire()
        if random_bush not in self.current_fires:
            self.current_fires.append(random_bush)

        return self.current_fires
    
    def get_on_fire(self):
        pass

    
    def draw(self):
        self.win.fill(WHITE)
        self.surf1.fill(WHITE)
        pygame.draw.rect(self.surf1,BLACK,(0,0,500,500),3,8)
        for bush in self.bushes:
            bush.draw(self.surf1)
        self.win.blit(self.surf1,(240,100))
    
    def run(self):
        events = pygame.event.get()
        for ev in events:
            if ev.type == pygame.QUIT:
                run = False
                pygame.quit()
        self.draw()
        pygame.display.update()


class Fire_Notifier(Node):
    def __init__(self,forest):
        super().__init__("Fire_Notifier")
        self.forest = forest
        self.publisher_ = self.create_publisher(Int64MultiArray,"Current_Fires",10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)                            

    def timer_callback(self):
        self.forest.run()
        msg = Int64MultiArray()
        msg.data = self.forest.trigger_fires()
        self.publisher_.publish(msg)

class Fire_Truck_POS_Subscriber(Node):
    def __init__(self,forest):
        super().__init__("Fire_Truck_POS_Subscriber")
        self.forest = forest
        self.subscriber_ = self.create_subscription(Coordinates,"Fire_Truck_Pos",self.subscriber_callback)                         

    def subscriber_callback(self,msg):
        pos = msg.x,msg.y
        self.forest.extingush_fire(pos)
        
        

def main(cols =20   ,rows = 12,args=None):
    rclpy.init(args=args)
    f = Forest_Node()
    Fire_Notifier_Node = Fire_Notifier(f)

    rclpy.spin(Fire_Notifier_Node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()