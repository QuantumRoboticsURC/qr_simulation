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
        self.subscribe = self.create_subscription(Twist,"cmd_vel",self.update_data)
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
    m = Main()
    rclpy.spin(m)
    #rclpy.destroy_node()
    #rclpy.shutdown()
    return m

if __name__ == "__main__":
    obj = main()
    pygame.init()
    pygame.display.set_caption('Simulación IMU')
    window_surface = pygame.display.set_mode((800, 800))
    manager = pygame_gui.UIManager((800, 800))
    """
    accelerometer_slider_x = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 520), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=manager)
    accelerometer_slider_y = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 560), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=manager)
    accelerometer_slider_z = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 600), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=manager)
    gyroscope_slider_x = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 640), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=manager)
    gyroscope_slider_y = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 680), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=manager)
    gyroscope_slider_z = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 720), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=manager)
    magnetometer_slider_x = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 760), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=manager)
    magnetometer_slider_y = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 800), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=manager)
    magnetometer_slider_z = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 840), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=manager)
    pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 500), (100, 20)), text='Acelerómetro X:', manager=manager)
    pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 540), (100, 20)), text='Acelerómetro Y:', manager=manager)
    pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 580), (100, 20)), text='Acelerómetro Z:', manager=manager)
    pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 620), (100, 20)), text='Giroscopio X:', manager=manager)
    pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 660), (100, 20)), text='Giroscopio Y:', manager=manager)
    pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 700), (100, 20)), text='Giroscopio Z:', manager=manager)
    pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 740), (100, 20)), text='Magnetómetro X:', manager=manager)
    pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 780), (100, 20)), text='Magnetómetro Y:', manager=manager)
    pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 820), (100, 20)), text='Magnetómetro Z:', manager=manager)
    coordinates_label = pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 860), (720, 20)), text='', manager=manager)
    """
    clock = pygame.time.Clock()
    running = True
    while running:
    dt = clock.tick(60)/1000.0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        manager.process_events(event)
    manager.update(dt)
    """
    ax = accelerometer_slider_x.get_current_value()
    ay = accelerometer_slider_y.get_current_value()
    az = accelerometer_slider_z.get_current_value()
    gx = gyroscope_slider_x.get_current_value()
    gy = gyroscope_slider_y.get_current_value()
    gz = gyroscope_slider_z.get_current_value()
    mx = magnetometer_slider_x.get_current_value()
    my = magnetometer_slider_y.get_current_value()
    mz = magnetometer_slider_z.get_current_value()
    """
    print(f"Coordenadas: {obj.x}, {obj.y}")
    print(f"Velocidad: {obj.vx}, {obj.vy}")
    #print(f"Acelerómetro: {ax}, {ay}, {az}")
    #print(f"Giroscopio: {gx}, {gy}, {gz}")
    #print(f"Magnetómetro: {mx}, {my}, {mz}")
    print(f"Angle: {obj.angle}")
    #velocity = np.array([obj.vx, obj.vy])
    #orientation = np.arctan2(obj.vy, obj.vx)
    #g = 9.81
    #gravity = np.array([0, -g])Coordenadas
    #print(f"Velocidad: {velocity}")
    #print(f"Orientación: {orientation}")
    #obj.update_position(ax, ay, dt)
    #data = None #TODO
    #obj.update_data(data)

    #print(f"Fuerza gravitacional: {gravity}\n\n")
    #coordinates_label.set_text(f"Latitud: {obj.lat:.2f}, Longitud: {obj.lon:.2f}")
    window_surface.fill((0, 0, 0))
    for x in range(0, 800, 80):
        pygame.draw.line(window_surface, (50, 50, 50), (x, 0), (x, 480))
    for y in range(0, 480, 80):
        pygame.draw.line(window_surface, (50, 50, 50), (0, y), (800, y))
    pygame.draw.circle(window_surface, (255, 255, 255), (obj.x, obj.y), 5)
    manager.draw_ui(window_surface)
    pygame.display.update()

    pygame.quit()
