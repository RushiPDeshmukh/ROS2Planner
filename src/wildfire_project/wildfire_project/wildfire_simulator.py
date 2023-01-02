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

#TO-DO
# 1. define a random patch() function this function defines a obstacle shape as numpy array
# 2. Define a rclpy node to subscribe to the topic "fire_truck_pos" 
# 3. Define a rclpy Server node to send the the obstacle field information to the player in a 1D array as a flatten adjMatrix
# 4. Define a rclpy node to publish the list of fires in the simulated forest on the topic "forest_fire_pos"


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


class Forest:
    def __init__(self,coverage = 10,width = 500,bush_width = 10) -> None:
        self.coverage = coverage
        self.width = width
        self.bush_width = bush_width
        self.number_of_bushes = 0
        self.bushes = []
        
        # self.grid_data = np.zeros((self.width//(self.bush_width),self.width//(self.bush_width))) CHANGE
    
    def can_extinguish(self,car_boundary):
        pass



    def create(self):
        """This function will randomly generate the Bush objects to populate the forest
        """
        self.bushes = []
        self.number_of_bushes = 0
        occupied = 0
        total_area = (self.width**2) #total area of forest
        coverage = occupied*100//total_area #current coverage of forest
        while coverage<self.coverage:
            print("Current Coverage: ",coverage)
            #random pos in the forest
            random_x = np.random.randint(0,self.width)
            random_y = np.random.randint(0,self.width)
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
                print("pos: ",bush.pos)
                print("shape:",bush.shape )
                self.number_of_bushes +=1
                occupied += shape[0]*shape[1]*self.bush_width**2
            coverage = occupied*100//total_area

    def is_collision(self,rect2):
        for bush in self.bushes:
            rect1 = bush.get_rect()
            if rect1.colliderect(rect2):
                return True
        return False

    def get_grid(self):
        return self.grid_data
    
    def trigger_fire(self,t):
        """This function will randomly choose a bush to set on fire.
           This function will choose a bush only if it is not already on fire.
        """
        pass
    
    def get_on_fire(self):
        pass

    
    def draw(self,surface):
        for bush in self.bushes:
            bush.draw(surface)


if __name__ == "__main__":
    
    BLACK = (0,0,0)
    WHITE = (255,255,255)
    win = pygame.display.set_mode((1000,700))
    surf1 = pygame.surface.Surface((500,500))
    
    run = True
    pos = (0,0)
    f = Forest()
    f.create()
   
    while run:
        win.fill(WHITE)
        surf1.fill(WHITE)
        pygame.draw.rect(surf1,BLACK,(0,0,500,500),3,8)
        events = pygame.event.get()
        for ev in events:
            if ev.type == pygame.QUIT:
                run = False
                pygame.quit()
        f.draw(surf1)
        win.blit(surf1,(240,100))
        pygame.display.update()
        pygame.time.wait(1000)
