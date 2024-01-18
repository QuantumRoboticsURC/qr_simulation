import pygame
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Int8
from geometry_msgs.msg import Twist

class Simulator(Node):

    def __init__(self):
        super().__init__('simulator')

        self.twist = Twist()
        self.arrived = False

        self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel", self.cmd_vel_callbak, 10)
        self.sub_arrived = self.create_subscription(Bool, "arrived", self.arrived_callbak, 10)

        pygame.init()
        pygame.display.set_caption('QR Simulación IMU')
        self.window_surface = pygame.display.set_mode((800, 800))
        self.obj = self.Object(200, 150)
        self.clock = pygame.time.Clock()
        self.reloj = self.create_timer(0.01, self.game)

    class Object:

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.lat = 0
            self.lon = 0
            self.vx = 0
            self.vy = 0
            self.theta = 0

        def update_position(self, twist, dt):
            self.theta+=twist.angular.z*dt
            self.vx = twist.linear.x * dt*np.cos(self.theta)*100
            self.vy = twist.linear.x * dt*np.sin(self.theta)*100
            self.theta = (self.theta+2*math.pi)%(2*math.pi)
            self.x += self.vx * dt
            self.y += self.vy * dt
            self.lat += self.vx * 0.001
            self.lon += self.vy * 0.001
        
        def print_status_log(self):
            print(f"Linear velocity = [x: {self.vx}, y: {self.vy}]")
            print(f"Theta angle = {self.theta}°")
            print(f"Coordinates = [lat: {self.lat}, lon: {self.lon}]")
    
    def game(self):
            self.obj.print_status_log()
            dt = self.clock.tick(60)/100.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
            self.obj.update_position(self.twist, dt)
            self.window_surface.fill((0, 0, 0))
            for x in range(0, 800, 80):
                pygame.draw.line(self.window_surface, (50, 50, 50), (x, 0), (x, 800))
            for y in range(0, 800, 80):
                pygame.draw.line(self.window_surface, (50, 50, 50), (0, y), (800, y))
            pygame.draw.circle(self.window_surface, (255, 255, 255), (self.obj.x, self.obj.y), 5)
            pygame.display.update()

    def cmd_vel_callbak(self,msg):
        self.twist = msg

    def arrived_callbak(self,msg):
        self.arrived = msg.data

  
def main(args=None):
    rclpy.init(args=args)
    m = Simulator()
    rclpy.spin(m)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
