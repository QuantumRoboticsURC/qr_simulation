import pygame
import sys
import math

# Inicializar Pygame
pygame.init()

# Configuración de la pantalla
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Espiral de Arquímedes")

# Colores
black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)

# Parámetros de la espiral
a = 10  # Factor de escala
b = 2   # Factor de apertura
theta_max = 2 * math.pi * 5  # Número de vueltas

# Parámetros de los puntos de navegación
distancia_entre_puntos = 20

def draw_spiral():
    for theta in range(0, int(theta_max * 100), 1):
        theta /= 100.0
        x = a * theta * math.cos(theta)
        y = a * theta * math.sin(theta)
        pygame.draw.circle(screen, white, (int(width / 2 + x), int(height / 2 - y)), 2)

def draw_navigation_points(distancia_entre_puntos):
    last_point = None
    last_theta = None
    for theta in range(0, int(theta_max * 100), distancia_entre_puntos):
        theta /= 100.0
        x = a * theta * math.cos(theta)
        y = a * theta * math.sin(theta)
        current_point = (int(width / 2 + x), int(height / 2 - y))

        if last_point is not None:
            pygame.draw.line(screen, red, last_point, current_point, 2)

        pygame.draw.circle(screen, red, current_point, 5)
        last_point = current_point
        last_theta = theta

# Bucle principal
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    screen.fill(black)

    draw_spiral()
    draw_navigation_points(distancia_entre_puntos)

    pygame.display.flip()
