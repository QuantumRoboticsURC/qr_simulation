import rclpy, pygame, math, bisect, numpy as np
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, Float64
from geometry_msgs.msg import Twist
from custom_interfaces.msg import TargetCoordinates, Coordinates
from .submodules.alvinxy import xy2ll

class Simulator(Node):

    def __init__(self):
        super().__init__('simulator')

        #Pygame variable members and configuration
        pygame.init()
        pygame.display.set_caption('Quantum Robotics | Rover in Action Simulation')
        self.WIDTH, self.HEIGHT = 800, 800
        self.window_surface = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        self.clock = pygame.time.Clock()

        #Class variable members
        self.twist = Twist()
        self.arrived = False
        self.state = None
        self.orglatitude = 19.5970212
        self.orglongitude = -99.227144
        self.rover = self.Rover(200, 150)
        self.archimedean_spiral = self.ArchimedeanSpiral()
        self.navigation_points = {"continuous": None, "discrete": None}

        #ROS 2 subscriptions and publishers
        self.sub_state = self.create_subscription(Int8,"/state", self.state_callback, 10)
        self.sub_cmd_vel = self.create_subscription(Twist,"/cmd_vel", self.cmd_vel_callback, 10)
        self.sub_arrived = self.create_subscription(Bool, "/arrived", self.arrived_callback, 10)
        self.pub_coordinates = self.create_publisher(Coordinates,'/coordinates',10)
        self.pub_angle = self.create_publisher(Float64,'/angle',10)
        
        #ROS 2 timer routines
        self.game_timer = self.create_timer(0.01, self.game)
    
    class Color(Enum):
        BLACK, WHITE, RED, GREEN, GRAY = (0, 0, 0), (255, 255, 255), (255, 0, 0), (0, 255, 0), (50, 50, 50)

    class Rover:

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.latitude = 0
            self.longitude = 0
            self.linear_velocity_x = 0
            self.linear_velocity_y = 0
            self.angle = 0

        def update_position(self, twist, dt):
            self.angle+=twist.angular.z*dt
            self.linear_velocity_x = twist.linear.x * dt*np.cos(self.angle)*100
            self.linear_velocity_y = twist.linear.x * dt*np.sin(self.angle)*100
            self.angle = (self.angle+2*math.pi)%(2*math.pi)
            self.x += self.linear_velocity_x * dt
            self.y += self.linear_velocity_y * dt
            self.latitude += self.linear_velocity_x * 0.001
            self.longitude += self.linear_velocity_y * 0.001
        
        def print_status_log(self):
            print(f"Position = [x: {self.x}, y: {self.y}]")
            print(f"Coordinates = [latitude: {self.latitude}, longitude: {self.longitude}]")
            print(f"Linear velocity = [x: {self.linear_velocity_x}, y: {self.linear_velocity_y}]")
            print(f"Angle = {self.angle}°")

    class ArchimedeanSpiral:

        def __init__(self):

            # Archimedean spiral variable members
            self.scale = 7
            self.turns = 3
            self.points_distance = 150
            self.continuous_points_number = 1000
        
        def get_continuous_scheme(self):
            partial_continuous_scheme = []
            angle_max = 2 * math.pi * self.turns
            points_number = int(angle_max * self.continuous_points_number)
            delta_angle = angle_max / points_number
            for index in range(points_number):
                angle = index * delta_angle
                partial_continuous_scheme.append((self.scale * angle * math.cos(angle), self.scale * angle * math.sin(angle)))
            return partial_continuous_scheme
        
        def get_discrete_scheme(self, continuous_scheme, particle):
            complete_continuous_scheme = []
            discrete_scheme = []
            for point in continuous_scheme:
                complete_continuous_scheme.append((int(particle.x + point[0]), int(particle.y - point[1])))
            distances = [0]
            for index in range(1, len(complete_continuous_scheme)):
                distance = math.sqrt((complete_continuous_scheme[index][0] - complete_continuous_scheme[index - 1][0])**2 +
                                    (complete_continuous_scheme[index][1] - complete_continuous_scheme[index - 1][1])**2)
                distances.append(distances[-1] + distance)
            for index in range(0, int(distances[-1]), self.points_distance):
                subindex = bisect.bisect(distances, index)
                current_point = complete_continuous_scheme[subindex]
                discrete_scheme.append(current_point)
            return discrete_scheme
    
    def game(self):
        dt = self.clock.tick(60)/100.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        self.draw_grid_scheme()
        if self.state == 0:
            self.autonomous_routine()
        self.rover.update_position(self.twist, dt)
        self.rover.print_status_log()
        pygame.draw.circle(self.window_surface, self.Color.WHITE.value, (self.rover.x, self.rover.y), 5)
        self.coordinates_callback()
        pygame.display.flip()
        pygame.display.update()
    
    def draw_grid_scheme(self):
        self.window_surface.fill(self.Color.BLACK.value)
        for x in range(0, self.WIDTH, self.WIDTH//10):
            pygame.draw.line(self.window_surface, self.Color.GRAY.value, (x, 0), (x, self.WIDTH))
        for y in range(0, self.HEIGHT, self.HEIGHT//10):
            pygame.draw.line(self.window_surface, self.Color.GRAY.value, (0, y), (self.HEIGHT, y))

    def autonomous_routine(self):
        if(not self.navigation_points["continuous"]):
            self.navigation_points["continuous"] = self.archimedean_spiral.get_continuous_scheme()
        if(not self.navigation_points["discrete"]):
            self.navigation_points["discrete"] = self.archimedean_spiral.get_discrete_scheme(self.navigation_points["continuous"], self.rover)
        for point in self.navigation_points["continuous"]:
            pygame.draw.circle(self.window_surface, self.Color.WHITE.value, (int(self.rover.x + point[0]), int(self.rover.y - point[1])), 1)
        for index, point in enumerate(self.navigation_points["discrete"]):
            pygame.draw.circle(self.window_surface, self.Color.RED.value, point, 5)
            if index != len(self.navigation_points["discrete"]) -1:
                pygame.draw.line(self.window_surface, self.Color.GREEN.value, point, self.navigation_points["discrete"][index + 1], 2)

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
        print(f"Coordinates = [latitude: {coordinates.latitude}, longitude: {coordinates.longitude}]")
        

def main(args=None):
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
