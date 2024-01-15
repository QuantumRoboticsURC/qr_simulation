import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8,Bool,Float64
from geometry_msgs.msg import Twist
from .submodules.alvinxy import *
import math
import numpy as np
from custom_interfaces.msg import TargetCoordinates,Coordinates
import pygame
import sys


class Simulation(Node):
    def __init__(self):
        super().__init__("Simulation")
        pygame.init()
        self.s = self.create_subscription(Twist,"/cmd_vel",self.update_data,10)
        self.coordinates = self.create_publisher(Coordinates,'/coordinates',10)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.angle = 0.0
        self.refresh_rate = 0.01
        self.orglat = 19.5970212
        self.orglong = -99.227144
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.image = pygame.Surface([30, 30])
        self.image.fill((250,0,0))
        self.rect = self.image.get_rect()
        self.screen = pygame.display.set_mode([500, 500])
        
        pygame.display.set_caption("Simulación 2")
        self.screen.fill((255, 255, 255))
        self.a=pygame.draw.circle(self.screen, (0, 0, 255), (self.x, self.y), 10)
        self.game = self.create_timer(0.001,self.main)
         

    def update_data(self,data):
        self.angle += data.angular.z*self.refresh_rate 
        self.angle = (self.angle+2*math.pi)%2*math.pi
        
        self.vx= data.linear.x
        self.vy = data.linear.y
        
        self.x += self.vx*self.refresh_rate*100
        self.y += self.vy*self.refresh_rate*100
        
        coords = Coordinates()
        lat,lon = xy2ll(self.x,self.y,self.orglat,self.orglong)
        coords.latitude = lat
        coords.longitude = lon
        self.coordinates.publish(coords)
    
    def main(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
        pos = (self.x,self.y)
        self.screen.blit(self.a,pos)    
        pygame.display.flip()
        
        
  
def main(args=None):
    rclpy.init(args=args)
    m = Simulation()
    rclpy.spin(m)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
        
        
        
        
    