#Program: Quantum Robotics Simulation | Rover in Action
#Version: 1.1a
#Developer: @emvivas (Emiliano Vivas Rodríguez)
#Contact: a01424732@tec.mx

import rclpy, pygame, math, bisect, copy, random, time, numpy as np
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, Float64
from geometry_msgs.msg import Twist
from custom_interfaces.msg import TargetCoordinates, Coordinates
from .submodules.alvinxy import xy2ll

class Simulator(Node):

    class Color(Enum):
        WHITE = (255, 255, 255)
        BLACK = (0, 0, 0)
        DARK_BLUE = (15, 28, 48)
        BLUE = (48, 74, 110)
        LIGHT_BLUE = (102, 141, 192)
        LIGHT_LIGHT_BLUE = (192, 208, 239)
        GRAY = (194, 198, 206)
        RED = (212, 122, 114)
        LIGHT_RED = (234, 200, 194)
        BROWN = (86, 63, 64)
        LIGHT_BROWN = (240, 229, 220)
        GREEN = (86, 191, 129)

    def __init__(self, dimension):
        super().__init__('simulator')

        #Class variable members
        self.twist = Twist()
        self.arrived = False
        self.state = None
        self.orglatitude = 19.5970212
        self.orglongitude = -99.227144
        self.rover = self.Rover((400, 400), ((19.25, 19.75), (-99, -99.50)), (0.05, 0.05), pygame.time.get_ticks())
        self.archimedean_spiral = self.ArchimedeanSpiral(7, 7, 150, 500)

        #Pygame variable members and configuration
        pygame.init()
        pygame.display.set_caption("Quantum Robotics Simulation | Rover in Action")
        self.DIMENSION = (dimension[0], dimension[1])
        self.window = pygame.display.set_mode(self.DIMENSION)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 15)
        self.title_label = self.font.render("Quantum Robotics Simulator", True, self.Color.BLACK.value, self.Color.WHITE.value)
        self.title_label_rect = self.title_label.get_rect()
        self.title_label_rect.center = (self.title_label_rect.width//2 + 10 , 10)

        #ROS 2 subscriptions and publishers
        self.sub_state = self.create_subscription(Int8,"/state", self.state_callback, 10)
        self.sub_cmd_vel = self.create_subscription(Twist,"/cmd_vel", self.cmd_vel_callback, 10)
        self.sub_arrived = self.create_subscription(Bool, "/arrived", self.arrived_callback, 10)
        self.pub_coordinates = self.create_publisher(Coordinates,'/coordinates',10)
        self.pub_target_coordinates = self.create_publisher(TargetCoordinates,'/target_coordinates',10)
        self.pub_angle = self.create_publisher(Float64,'/angle',10)
        
        #ROS 2 timer routines
        self.game_timer = self.create_timer(0.01, self.game)

    class Rover:

        def __init__(self, coordinates, coordinates_range, coordinates_offset, last_time_rotating):
            self.x, self.y = coordinates[0], coordinates[1]
            self.COORDINATES_RANGE = (coordinates_range[0], coordinates_range[1])
            self.COORDINATES_OFFSET = (coordinates_offset[0], coordinates_offset[1])
            self.latitude, self.longitude = self.xy_to_latlon(self.x, self.y)
            self.linear_velocity_x = 0
            self.linear_velocity_y = 0
            self.autonomous_linear_velocity_magnitude = 3
            self.angle = 0
            self.navigation_points = None
            self.navigation_point = None
            self.navigation_point_uncertainty = 10
            self.changed = True
            self.is_rotating = False
            self.last_time_rotating = last_time_rotating
            self.delay_time_rotating = 5000
        
        def xy_to_latlon(self, x, y):
            normalized_x = self.COORDINATES_RANGE[0][0] + (x / 800) * (self.COORDINATES_RANGE[0][1] - self.COORDINATES_RANGE[0][0])
            normalized_y = self.COORDINATES_RANGE[1][0] + (y / 800) * (self.COORDINATES_RANGE[1][1] - self.COORDINATES_RANGE[1][0])
            latitude = normalized_x + self.COORDINATES_OFFSET[1]
            longitude = normalized_y + self.COORDINATES_OFFSET[0]
            return latitude, longitude
        
        def latlon_to_xy(self, latitude, longitude):
            normalized_y = latitude - self.COORDINATES_OFFSET[0]
            normalized_x = longitude - self.COORDINATES_OFFSET[1]
            x = (normalized_x - self.COORDINATES_RANGE[0][0]) / (self.COORDINATES_RANGE[0][1] - self.COORDINATES_RANGE[0][0]) * 800
            y = (normalized_y - self.COORDINATES_RANGE[1][0]) / (self.COORDINATES_RANGE[1][1] - self.COORDINATES_RANGE[1][0]) * 800
            return x, y
        
        def set_linear_velocity(self, linear_velocity_x, linear_velocity_y):
            self.linear_velocity_x = linear_velocity_x
            self.linear_velocity_y = linear_velocity_y

        def update_position(self, dt, twist = None):
            if twist:
                if self.changed:
                    self.linear_velocity_x += twist.linear.x
                    self.linear_velocity_y += twist.linear.y
                    self.changed = False
                if twist.linear.x == 0 and twist.linear.y == 0 and twist.angular.z == 0:
                    self.changed = True
            self.angle=math.degrees(math.atan2(self.linear_velocity_x, self.linear_velocity_y))
            self.x += self.linear_velocity_x * dt
            self.y += self.linear_velocity_y * dt
            self.latitude, self.longitude = self.xy_to_latlon(self.x, self.y)
        
        def follow_navigation_points(self, navigation_points = []):
            if not self.navigation_points or len(self.navigation_points)==0:
                self.navigation_points = navigation_points
            if self.navigation_points and len(self.navigation_points) > 0:
                if not self.navigation_point or (
                        self.x >= self.navigation_point[0] - self.navigation_point_uncertainty and self.x <= self.navigation_point[0] + self.navigation_point_uncertainty and
                        self.y >= self.navigation_point[1] - self.navigation_point_uncertainty and self.y <= self.navigation_point[1] + self.navigation_point_uncertainty
                ):
                    self.is_rotating = True
                    self.navigation_point = self.navigation_points.pop(0)
                    target_x = self.navigation_point[0] + random.uniform(-self.navigation_point_uncertainty, self.navigation_point_uncertainty)
                    target_y = self.navigation_point[1] + random.uniform(-self.navigation_point_uncertainty, self.navigation_point_uncertainty)
                    angle = math.atan2(target_y - self.y, target_x - self.x)
                    self.set_linear_velocity(self.autonomous_linear_velocity_magnitude * math.cos(angle), self.autonomous_linear_velocity_magnitude * math.sin(angle))

        def print_status_log(self):
            print(f"Position = [x: {self.x}, y: {self.y}]")
            print(f"Coordinates = [lat: {self.latitude}, lon: {self.longitude}]")
            print(f"Linear velocity = [x: {self.linear_velocity_x}, y: {self.linear_velocity_y}]")
            print(f"Angle = {self.angle}°")

    class ArchimedeanSpiral:

        def __init__(self, scale, turns, points_distance, continuous_points_number):

            # Archimedean spiral variable members
            self.scale = scale
            self.turns = turns
            self.points_distance = points_distance
            self.continuous_points_number = continuous_points_number
            self.navigation_points = {"continuous": None, "discrete": None}
        
        class SchemeModalityDraw(Enum):
            ALL, CONTINUOUS, DISCRETE, NONE = 0, 1, 2, 3
        
        def calculate_continuous_scheme(self, particle):
            self.navigation_points["continuous"] = []
            angle_max = 2 * math.pi * self.turns
            points_number = int(angle_max * self.continuous_points_number)
            delta_angle = angle_max / points_number
            for index in range(points_number):
                angle = index * delta_angle
                self.navigation_points["continuous"].append((int(self.scale * angle * math.cos(angle) + particle.x), int(self.scale * angle * math.sin(angle) + particle.y)))
        
        def calculate_discrete_scheme(self):
            self.navigation_points["discrete"] = []
            distances = [0]
            for index in range(1, len(self.navigation_points["continuous"])):
                distance = math.sqrt((self.navigation_points["continuous"][index][0] - self.navigation_points["continuous"][index - 1][0])**2 +
                                    (self.navigation_points["continuous"][index][1] - self.navigation_points["continuous"][index - 1][1])**2)
                distances.append(distances[-1] + distance)
            for index in range(0, int(distances[-1]), self.points_distance):
                subindex = bisect.bisect(distances, index)
                current_point = self.navigation_points["continuous"][subindex]
                self.navigation_points["discrete"].append(current_point)
        
        def get_continuous_scheme(self, particle = None):
            if particle and not self.navigation_points["continuous"]:
                self.calculate_continuous_scheme(particle)
            return self.navigation_points["continuous"]
        
        def get_discrete_scheme(self, particle = None):
            if not self.navigation_points["discrete"]:
                if particle and not self.navigation_points["continuous"]:
                    self.calculate_continuous_scheme(particle)
                self.calculate_discrete_scheme()
            return self.navigation_points["discrete"]
        
        def clear_navigation_points(self):
            self.navigation_points = {"continuous": None, "discrete": None}
    
    def draw_stage(self):
        self.window.fill(self.Color.WHITE.value)
        for x in range(0, self.DIMENSION[0], self.DIMENSION[0]//50):
            pygame.draw.line(self.window, self.Color.GRAY.value, (x, 0), (x, self.DIMENSION[0]))
        for y in range(0, self.DIMENSION[1], self.DIMENSION[1]//50):
            pygame.draw.line(self.window, self.Color.GRAY.value, (0, y), (self.DIMENSION[1], y))
        position_label = self.font.render(f"Position = [x: {self.rover.x:,.3f}, y: {self.rover.y:,.3f}]", True, self.Color.BLACK.value, self.Color.WHITE.value)
        position_label_rect = position_label.get_rect()
        position_label_rect.center = (position_label_rect.width//2 + 10 , 25)
        coordinates_label = self.font.render(f"Coordinates = [lat: {self.rover.latitude:,.3f}, lon: {self.rover.longitude:,.3f}]", True, self.Color.BLACK.value, self.Color.WHITE.value)
        coordinates_label_rect = coordinates_label.get_rect()
        coordinates_label_rect.center = (coordinates_label_rect.width//2 + 10 , 35)
        linear_velocity_label = self.font.render(f"Linear velocity = [x: {self.rover.linear_velocity_x:,.3f}, y: {self.rover.linear_velocity_y:,.3f}]", True, self.Color.BLACK.value, self.Color.WHITE.value)
        linear_velocity_label_rect = linear_velocity_label.get_rect()
        linear_velocity_label_rect.center = (linear_velocity_label_rect.width//2 + 10 , 45)
        angle_label = self.font.render(f"Angle = {self.rover.angle:,.3f}°", True, self.Color.BLACK.value, self.Color.WHITE.value)
        angle_label_rect = angle_label.get_rect()
        angle_label_rect.center = (angle_label_rect.width//2 + 10 , 55)
        self.window.blit(self.title_label, self.title_label_rect)
        self.window.blit(position_label, position_label_rect)
        self.window.blit(coordinates_label, coordinates_label_rect)
        self.window.blit(linear_velocity_label, linear_velocity_label_rect)
        self.window.blit(angle_label, angle_label_rect)

    def draw_autonomous_routine(self, scheme_modality_draw):
        if scheme_modality_draw.value == self.ArchimedeanSpiral.SchemeModalityDraw.NONE.value:
            self.archimedean_spiral.calculate_continuous_scheme(self.rover)
            self.archimedean_spiral.calculate_discrete_scheme()
        else:
            if scheme_modality_draw.value == self.ArchimedeanSpiral.SchemeModalityDraw.CONTINUOUS.value or scheme_modality_draw.value == self.ArchimedeanSpiral.SchemeModalityDraw.ALL.value:
                for point in self.archimedean_spiral.get_continuous_scheme(self.rover):
                    pygame.draw.circle(self.window, self.Color.LIGHT_LIGHT_BLUE.value, point, 1)
            if scheme_modality_draw.value == self.ArchimedeanSpiral.SchemeModalityDraw.DISCRETE.value or scheme_modality_draw.value == self.ArchimedeanSpiral.SchemeModalityDraw.ALL.value:
                for index, point in enumerate(self.archimedean_spiral.get_discrete_scheme(self.rover)):
                    if index != len(self.archimedean_spiral.get_discrete_scheme()) -1:
                        pygame.draw.line(self.window, self.Color.GREEN.value, point, self.archimedean_spiral.get_discrete_scheme()[index + 1], 2)
                    pygame.draw.circle(self.window, self.Color.LIGHT_BLUE.value, point, 5)

    def game(self):
        dt = self.clock.tick(60)/100.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        self.draw_stage()
        if self.state == 0:
            self.draw_autonomous_routine(self.ArchimedeanSpiral.SchemeModalityDraw.ALL)
            if self.rover.is_rotating:
                now = pygame.time.get_ticks()
                if now - self.rover.last_time_rotating >= self.rover.delay_time_rotating:
                    self.rover.last_time_rotating = now
                    self.rover.is_rotating = False
            else:
                self.rover.follow_navigation_points(copy.copy(self.archimedean_spiral.get_discrete_scheme()))
                self.rover.update_position(dt)
        else:
            self.rover.update_position(dt, self.twist)
        self.rover.print_status_log()
        pygame.draw.circle(self.window, self.Color.BLACK.value, (self.rover.x, self.rover.y), 5)
        self.coordinates_callback()
        self.target_coordinates_callback()
        pygame.display.flip()
        pygame.display.update()

    #ROS 2 callbacks
    def cmd_vel_callback(self,msg):
        self.twist = msg
    
    def state_callback(self,msg):
        self.state = msg.data

    def arrived_callback(self,msg):
        self.arrived = msg.data
        
    def coordinates_callback(self):
        coordinates = Coordinates()
        coordinates.latitude,coordinates.longitude = xy2ll(self.rover.x,self.rover.y,self.orglatitude,self.orglongitude)
        self.pub_coordinates.publish(coordinates)
        angle = Float64()
        angle.data = self.rover.angle
        self.pub_angle.publish(angle)
        print(f"Coordinates xy2ll = [lat: {coordinates.latitude}, lon: {coordinates.longitude}]")
    
    def target_coordinates_callback(self):
        target_coordinates = TargetCoordinates()
        target_coordinates.latitude,target_coordinates.longitude = xy2ll(self.rover.navigation_point[0] if self.rover.navigation_point else 0,self.rover.navigation_point[1] if self.rover.navigation_point else 0,self.orglatitude,self.orglongitude)
        self.pub_target_coordinates.publish(target_coordinates)
        print(f"Target Coordinates xy2ll = [lat: {target_coordinates.latitude}, lon: {target_coordinates.longitude}]")


def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator((800, 800))
    rclpy.spin(simulator)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
