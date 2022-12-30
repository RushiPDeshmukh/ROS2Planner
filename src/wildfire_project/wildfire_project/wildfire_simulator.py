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
from math import *
import os
import time

#TO-DO
# 1. define a random patch() function this function defines a obstacle shape as numpy array
# 2. Define a rclpy node to communicate with the player
# 3. Define a rclpy node to send the the obstacle field information to the player 

class Bush:
    def __init__(self,pos, tree_width = 10) -> None:
        self.pos = pos #stores the topLeft corner pos of the forest patch
        self.x = pos[0]
        self.y = pos[1]
        self.tree_width = tree_width #this variable sets the width of single unit of the patch.
        self.data,self.shape = random_patch() #this function returns a random forest shape and its shape variable
        self.type = "bush"
        self.surface = pygame.image.load(os.path.abspath('src/utils/bush.png'))
        self.surface = pygame.transform.scale(self.surface,(self.tree_width,self.tree_width))
        self.rects = []
        self.fire_start_time = 0

    def get_data(self):
        return self.pos,self.data,self.shape

    def change_shape(self):
        self.data,self.shape = random_patch()
        
        return self.data,self.shape

    def set_fire(self,t):
        self.type = "fire"
        self.fire_start_time = t
        self.surface = pygame.image.load(os.path.abspath('src/utils/fire.png'))
        self.surface = pygame.transform.scale(self.surface,(self.tree_width,self.tree_width))

    def extinguish_fire(self,t):
        self.type = "bush"
        self.fire_start_time = t
        self.surface = pygame.image.load(os.path.abspath('src/utils/bush.png'))
        self.surface = pygame.transform.scale(self.surface,(self.tree_width,self.tree_width))
    def on_fire(self):
        return self.type == 'fire'
    
    def get_rects(self):
        x,y = self.pos
        self.rects = []
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):
                if self.data[i][j] == 1:
                    rect = self.surface.get_rect()
                    rect.topleft = (x+(self.tree_width*i),y+(self.tree_width*j))
                    self.rects.append(rect)
        return self.rects
    def nearby_bush(self):
        area = pygame.Rect(0,0,120,120)
        area.center = (self.x + self.shape[0]*self.tree_width//2, self.y + self.shape[1]*self.tree_width//2)
        return area
    def check_nearby_bush(self,bush):
        area = self.nearby_bush()
        indices =  pygame.Rect.collidelistall(area,bush.get_rects())
        if len(indices) == len(bush.get_rects()):
            return True
        return False
    def spread_fire(self,bushes,t):
        if t - self.fire_start_time >= 20:
            self.fire_start_time = t
            for bush in bushes:
                if self.check_nearby_bush(bush):
                    bush.set_fire(t)
    
    def check_collision(self,rect_list):
        rects = self.get_rects()
        collision = False
        for rect in rects:
                index = pygame.Rect.collidelist(rect,rect_list)
                if index != -1:
                    collision == True
                    break
        return collision
    
    def nearby_car(self):
        area = pygame.Rect(0,0,self.shape[0]*self.tree_width+40,self.shape[1]*self.tree_width*40)
        area.center = (self.x + self.shape[0]*self.tree_width//2, self.y + self.shape[1]*self.tree_width//2)
        return area
    def is_nearby(self,car_rect):
        area = self.nearby_car()
        if pygame.Rect.colliderect(area,car_rect):
            return True
        return False
    def draw(self,win):
        x,y = self.pos
        # rect = self.nearby_bush()
        # pygame.draw.rect(win,BLUE,rect,width = 2)
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):
                if self.data[i][j] == 1:
                    win.blit(self.surface,(x+(self.tree_width*i),y+(self.tree_width*j)))


class Forest:
    def __init__(self,coverage = 20,width = 500,bush_width = 10) -> None:
        self.coverage = coverage
        self.width = width
        self.bush_width = bush_width
        self.number_of_bushes = 0
        self.bushes = []
        self.grid_data = np.zeros((self.width//(self.bush_width),self.width//(self.bush_width)))

    def is_collision(self,rect_list):
        collision =False
        for bush in self.bushes:
            collision = bush.check_collision(rect_list)
            if collision == True:
                break
        return collision
    
    def can_extinguish(self,rect,check_fire_only = True):
        bush_list = []
        for bush in self.bushes:
            if check_fire_only:
                if bush.is_nearby(rect) and bush.on_fire():
                    bush_list.append(bush)
            else:
                if bush.is_nearby(rect):
                    bush_list.append(bush)
        return bush_list

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
            random_x = np.random.randint(0,self.width//(self.bush_width*3))*(self.bush_width*3)
            random_y = np.random.randint(0,self.width//(self.bush_width*3))*(self.bush_width*3)
            #generate the bush object at this location
            bush = Bush((random_x,random_y),self.bush_width)
            _,bush_data,shape = bush.get_data()
            counter = 7
            while not ((random_x+shape[0]*self.bush_width < self.width) and (random_y+shape[1]*self.bush_width < self.width)) and counter != 0:
                bush_data,shape = bush.change_shape()
                counter -=1
            
            if not ((random_x+shape[0]*self.bush_width < self.width) and (random_y+shape[1]*self.bush_width < self.width)):
                continue

            rect_list = bush.get_rects()
            #check if the bush fits inside the forest and add to the list of bushes 
            if not self.is_collision(rect_list):
                self.bushes.append(bush)
                self.number_of_bushes +=1
                occupied += bush_data.sum()*self.bush_width**2
                for i in range(shape[0]):
                    for j in range(shape[1]):
                        x = i + random_x//(self.bush_width)
                        y = j + random_y//(self.bush_width)
                        self.grid_data[x][y] = bush_data[i][j]
            coverage = occupied*100//total_area
        
    def get_grid(self):
        return self.grid_data
    
    def trigger_fire(self,t):
        """This function will randomly choose a bush to set on fire.
           This function will choose a bush only if it is not already on fire.
        """
        done = False
        while not done:
            random_index = np.random.randint(0,len(self.bushes))
            if not self.bushes[random_index].on_fire():
                print("fire!")
                self.bushes[random_index].set_fire(t)
                done = True
                goal = (self.bushes[random_index].pos[0]//50,self.bushes[random_index].pos[1]//50,0)
        return goal,self.bushes[random_index]
    def get_on_fire(self):
        fire_list = []
        for bush in self.bushes:
            if bush.on_fire:
                fire_list.append((bush,bush.fire_start_time))
        return fire_list

    
    def spread_fire(self,t):
        for bush in self.bushes:
            if bush.on_fire():
                bush.spread_fire(self.bushes,t)
    
    def draw(self,win):
        for bush in self.bushes:
            bush.draw(win)


if __name__ == "__main__":
    width_win = 500
    
    win = pygame.display.set_mode((width_win,width_win))
    pygame.display.set_caption("Title")
    pygame.font.init()
    font = pygame.font.Font('freesansbold.ttf',10)
    text = font.render("T",True,BLACK)
    textrect = text.get_rect()
    textrect.topleft = (450,10)
    
    run = True
    pos = (0,0)
    f = Forest()
    f.create()
    t_last = time.time()
    t = 0           
    while t <= 300 and run:
        win.fill(WHITE)
        events = pygame.event.get()
        for ev in events:
            if ev.type == pygame.QUIT:
                run = False
                pygame.quit()
        if t%30 == 0:
            f.trigger_fire(t)
        #Spreading fire is too quick
        if t%20 == 0:
             f.spread_fire(t)
        f.draw(win)
        text = font.render("Time: "+str(t),True,BLACK)
        win.blit(text,textrect) 
        pygame.display.update()
        pygame.time.wait(1000)
        t += int(time.time() - t_last)
        t_last = time.time()
        print(t)


