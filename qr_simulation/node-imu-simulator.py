#!/usr/bin/env python
import pygame
import pygame_gui
import numpy as np
import rclpy
from rclpy.node import Node


class IMUSimulator(Node):

    class Object:
            def __init__(self, x, y):
                self.x = x
                self.y = y
                self.lat = 0
                self.lon = 0
                self.vx = 0
                self.vy = 0
            def update_position(self, dx, dy, dt):
                self.vx += dx * dt
                self.vy += dy * dt
                self.x += self.vx * dt
                self.y += self.vy * dt
                self.lat += self.vx * 0.001
                self.lon += self.vy * 0.001

    def __init__(self):
        super().__init__('imu_simulator')
        pygame.init()
        pygame.display.set_caption('Simulación IMU')
        self.window_surface = pygame.display.set_mode((800, 800))
        self.manager = pygame_gui.UIManager((800, 800))

        self.obj = self.Object(200, 150)
        self.accelerometer_slider_x = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 520), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=self.manager)
        self.accelerometer_slider_y = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 560), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=self.manager)
        self.accelerometer_slider_z = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 600), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=self.manager)
        self.gyroscope_slider_x = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 640), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=self.manager)
        self.gyroscope_slider_y = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 680), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=self.manager)
        self.gyroscope_slider_z = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 720), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=self.manager)
        self.magnetometer_slider_x = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 760), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=self.manager)
        self.magnetometer_slider_y = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 800), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=self.manager)
        self.magnetometer_slider_z = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((40, 840), (720, 20)), start_value=0, value_range=(-5.0, 5.0), manager=self.manager)
        pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 500), (100, 20)), text='Acelerómetro X:', manager=self.manager)
        pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 540), (100, 20)), text='Acelerómetro Y:', manager=self.manager)
        pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 580), (100, 20)), text='Acelerómetro Z:', manager=self.manager)
        pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 620), (100, 20)), text='Giroscopio X:', manager=self.manager)
        pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 660), (100, 20)), text='Giroscopio Y:', manager=self.manager)
        pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 700), (100, 20)), text='Giroscopio Z:', manager=self.manager)
        pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 740), (100, 20)), text='Magnetómetro X:', manager=self.manager)
        pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 780), (100, 20)), text='Magnetómetro Y:', manager=self.manager)
        pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 820), (100, 20)), text='Magnetómetro Z:', manager=self.manager)
        coordinates_label = pygame_gui.elements.UILabel(relative_rect=pygame.Rect((40, 860), (720, 20)), text='', manager=self.manager)
        self.clock = pygame.time.Clock()
        running = True
        while running:
            dt = self.clock.tick(60)/1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                self.manager.process_events(event)
            self.manager.update(dt)
            ax = self.accelerometer_slider_x.get_current_value()
            ay = self.accelerometer_slider_y.get_current_value()
            az = self.accelerometer_slider_z.get_current_value()
            gx = self.gyroscope_slider_x.get_current_value()
            gy = self.gyroscope_slider_y.get_current_value()
            gz = self.gyroscope_slider_z.get_current_value()
            mx = self.magnetometer_slider_x.get_current_value()
            my = self.magnetometer_slider_y.get_current_value()
            mz = self.magnetometer_slider_z.get_current_value()
            print(f"Coordenadas: {self.obj.lat}, {self.obj.lon}")
            print(f"Acelerómetro: {ax}, {ay}, {az}")
            print(f"Giroscopio: {gx}, {gy}, {gz}")
            print(f"Magnetómetro: {mx}, {my}, {mz}")
            velocity = np.array([self.obj.vx, self.obj.vy])
            orientation = np.arctan2(self.obj.vy, self.obj.vx)
            g = 9.81
            gravity = np.array([0, -g])
            print(f"Velocidad: {velocity}")
            print(f"Orientación: {orientation}")
            self.obj.update_position(ax, ay, dt)
            print(f"Fuerza gravitacional: {gravity}\n\n")
            coordinates_label.set_text(f"Latitud: {self.obj.lat:.2f}, Longitud: {self.obj.lon:.2f}")
            self.window_surface.fill((0, 0, 0))
            for x in range(0, 800, 80):
                pygame.draw.line(self.window_surface, (50, 50, 50), (x, 0), (x, 480))
            for y in range(0, 480, 80):
                pygame.draw.line(self.window_surface, (50, 50, 50), (0, y), (800, y))
            pygame.draw.circle(self.window_surface, (255, 255, 255), (self.obj.x, self.obj.y), 5)
            self.manager.draw_ui(self.window_surface)
            pygame.display.update()
        #pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = IMUSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.is_shutdown()


if __name__ == "__main__":
    main()
