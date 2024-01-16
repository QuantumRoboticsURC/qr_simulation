import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8,Bool,Float64
from geometry_msgs.msg import Twist
from .submodules.alvinxy import *
import math
import numpy as np
from custom_interfaces.msg import TargetCoordinates,Coordinates
import pygame as pg
from pygame.math import Vector2
import sys

class Simulation(Node):
    def __init__(self):
        super().__init__("Simulation")
        pg.init()
        self.s = self.create_subscription(Twist,"/cmd_vel",self.update_data,10)
        self.coordinates = self.create_publisher(Coordinates,'/coordinates',10)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.angle = 0.0
        self.refresh_rate = 0.001
        self.orglat = 19.5970212
        self.orglong = -99.227144
        self.x = 250.0
        self.y = 250.0
        self.vx = 0.0
        self.vy = 0.0
        self.img = pg.image.load('/home/iker/ros2_ws/src/qr_simulation/qr_simulation/speed.jpg')
        
        self.win = pg.display.set_mode((500, 500)) 
        
        # set the pygame window name  
        pg.display.set_caption("Moving rectangle") 
        self.timer = self.create_timer(0.001,self.main)
        
         

    def update_data(self,data):
        if(data.angular.z==0.0):
            self.angle = 0.0
        else:
            self.angle += data.angular.z*self.refresh_rate 
        self.angle = (np.degrees(self.angle+2*math.pi)%360)
        
        self.vx= data.linear.x#*np.cos(self.angle)
        self.vy = data.linear.x#*np.sin(self.angle)
        
        self.x += self.vx*np.cos(self.angle)*100
        self.y += self.vy*np.sin(self.angle)*100
        
        coords = Coordinates()
        lat,lon = xy2ll(self.x,self.y,self.orglat,self.orglong)
        coords.latitude = lat
        coords.longitude = lon
        self.coordinates.publish(coords)
    
    def main(self):
        for event in pg.event.get(): 
            
            # if event object type is QUIT   
            # then quitting the pygame   
            # and program both.   
            if event.type == pg.QUIT: 
                
                # it will make exit the while loop  
                run = False
            self.win.fill((0, 0, 0)) 
      
            # drawing object on screen which is rectangle here  
            print(self.angle)
            imgrot = pg.transform.rotate(self.img, (-self.angle)) #le da el valor de rotación a la imagen con base en el angulo de odometry
            
            self.win.blit(imgrot, (int(self.y),int(self.x))) 
            
            # it refreshes the window 
            pg.display.update()  
        
  
def main(args=None):
    rclpy.init(args=args)
    m = Simulation()
    rclpy.spin(m)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
        
        
        
        
    