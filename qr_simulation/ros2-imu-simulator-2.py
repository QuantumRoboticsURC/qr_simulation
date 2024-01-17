import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8,Bool,Float64
from geometry_msgs.msg import Twist
from .submodules.alvinxy import *
import math
import numpy as np
from custom_interfaces.msg import TargetCoordinates,Coordinates

import pygame
import pygame_gui
import numpy as np

class Main(Node):
    def __init__(self):
        
        pygame.init()
        pygame.display.set_caption('Simulación IMU')
        window_surface = pygame.display.set_mode((800, 800))
        manager = pygame_gui.UIManager((800, 800))

        self.subscribe = self.create_subscription(Twist,"cmd_vel",self.update_velocity)
        self.coordinates = self.create_publisher(Coordinates,'/coordinates', 10)
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
        
    
    def update_data(self,data):
        self.angle += data.angular.z*self.refresh_rate 
        self.angle = (self.angle+2*math.pi)%2*math.pi
        
        self.vx= data.linear.x*np.cos(self.angle)
        self.vy = data.linear.x*np.sin(self.angle)
        
        self.x += self.vx*self.refresh_rate
        self.y += self.vy*self.refresh_rate
        
        coords = Coordinates()
        lat,lon = xy2ll(self.x,self.y,self.orglat,self.orglong)
        coords.latitude = lat
        coords.longitude = lon
        self.coordinates.publish(coords)
        
        
  
def main(args=None):
    rclpy.init(args=args)
    obj = Main()
    clock = pygame.time.Clock()
    running = True
    while running:
    dt = clock.tick(60)/1000.0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        manager.process_events(event)
    manager.update(dt)
    print(f"Coordenadas: {obj.x}, {obj.y}")
    print(f"Velocidad: {obj.vx}, {obj.vy}")
    print(f"Angle: {obj.angle}")
    window_surface.fill((0, 0, 0))
    for x in range(0, 800, 80):
        pygame.draw.line(window_surface, (50, 50, 50), (x, 0), (x, 480))
    for y in range(0, 480, 80):
        pygame.draw.line(window_surface, (50, 50, 50), (0, y), (800, y))
    pygame.draw.circle(window_surface, (255, 255, 255), (obj.x, obj.y), 5)
    manager.draw_ui(window_surface)
    pygame.display.update()
    #pygame.quit()
    rclpy.spin(obj)
    #rclpy.destroy_node()
    #rclpy.shutdown()

if __name__ == "__main__":
    main()
