import pygame
import pygame_gui
import numpy as np

pygame.init()
pygame.display.set_caption('Simulación IMU')
window_surface = pygame.display.set_mode((800, 800))
manager = pygame_gui.UIManager((800, 800))

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

obj = Object(200, 150)
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
clock = pygame.time.Clock()
running = True
while running:
    dt = clock.tick(60)/1000.0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        manager.process_events(event)
    manager.update(dt)
    ax = accelerometer_slider_x.get_current_value()
    ay = accelerometer_slider_y.get_current_value()
    az = accelerometer_slider_z.get_current_value()
    gx = gyroscope_slider_x.get_current_value()
    gy = gyroscope_slider_y.get_current_value()
    gz = gyroscope_slider_z.get_current_value()
    mx = magnetometer_slider_x.get_current_value()
    my = magnetometer_slider_y.get_current_value()
    mz = magnetometer_slider_z.get_current_value()
    print(f"Coordenadas: {obj.lat}, {obj.lon}")
    print(f"Acelerómetro: {ax}, {ay}, {az}")
    print(f"Giroscopio: {gx}, {gy}, {gz}")
    print(f"Magnetómetro: {mx}, {my}, {mz}")
    velocity = np.array([obj.vx, obj.vy])
    orientation = np.arctan2(obj.vy, obj.vx)
    g = 9.81
    gravity = np.array([0, -g])
    print(f"Velocidad: {velocity}")
    print(f"Orientación: {orientation}")
    obj.update_position(ax, ay, dt)
    print(f"Fuerza gravitacional: {gravity}\n\n")
    coordinates_label.set_text(f"Latitud: {obj.lat:.2f}, Longitud: {obj.lon:.2f}")
    window_surface.fill((0, 0, 0))
    for x in range(0, 800, 80):
        pygame.draw.line(window_surface, (50, 50, 50), (x, 0), (x, 480))
    for y in range(0, 480, 80):
        pygame.draw.line(window_surface, (50, 50, 50), (0, y), (800, y))
    pygame.draw.circle(window_surface, (255, 255, 255), (obj.x, obj.y), 5)
    manager.draw_ui(window_surface)
    pygame.display.update()
pygame.quit()
