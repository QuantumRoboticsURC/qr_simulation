import rclpy, math, bisect
from rclpy.node import Node
from geometry_msgs.msg import Point
from custom_interfaces.msg import Coordinates, NavigationPoints
from std_msgs.msg import Int8
from .submodules.alvinxy import ll2xy, xy2ll

class Searching(Node):

    def __init__(self):
        super().__init__('searching')

        # Archimedean spiral variable members
        self.orglatitude = 19.5970212
        self.orglongitude = -99.227144
        self.state = None
        self.once = False
        self.coordinates = Coordinates()
        self.point = Point()
        self.scale = 7
        self.turns = 7
        self.points_distance = 150
        self.continuous_points_number = 500
        self.continuous_route = []
        self.discrete_route = []
        self.navigation_points = []
        self.sub_coordinates = self.create_subscription(Coordinates, '/coordinates', self.coordinates_callback, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.pub_navigation_points = self.create_publisher(NavigationPoints, '/navigation_points', 10)
        self.sub_state = self.create_subscription(Int8, "/state", self.state_callback, 10)

    def coordinates_callback(self,msg):
        self.coordinates.latitude,self.coordinates.longitude = msg.latitude, msg.longitude
        self.point.x, self.point.y = ll2xy(msg.latitude, msg.longitude,self.orglatitude,self.orglongitude)
    
    def state_callback(self,msg):
        self.state = msg.data
    
    def timer_callback(self):
        if self.state == 0 and not self.once:
            self.calculate_continuous_scheme()
            self.calculate_discrete_scheme()
            self.once = True
        self.pub_navigation_points.publish(self.navigation_points)

    def calculate_continuous_scheme(self):
        self.continuous_route = []
        angle_max = 2 * math.pi * self.turns
        points_number = int(angle_max * self.continuous_points_number)
        delta_angle = angle_max / points_number
        for index in range(points_number):
            angle = index * delta_angle
            self.continuous_route.append((int(self.scale * angle * math.cos(angle) + self.point.x), int(self.scale * angle * math.sin(angle) + self.point.y)))
        
    def calculate_discrete_scheme(self):
        self.discrete_route = []
        distances = [0]
        for index in range(1, len(self.continuous_route)):
            distance = math.sqrt((self.continuous_route[index][0] - self.continuous_route[index - 1][0])**2 +
                                (self.continuous_route[index][1] - self.continuous_route[index - 1][1])**2)
            distances.append(distances[-1] + distance)
        for index in range(0, int(distances[-1]), self.points_distance):
            subindex = bisect.bisect(distances, index)
            current_point = self.continuous_route[subindex]
            self.discrete_route.append(current_point)
            coordinates = Coordinates()
            coordinates.latitude,coordinates.longitude = xy2ll(current_point[0],current_point[1],self.orglatitude,self.orglongitude)
            self.navigation_points.append(coordinates)
    
    def get_continuous_scheme(self):
        if len(self.continuous_route) == 0:
            self.calculate_continuous_scheme()
        return self.continuous_route
    
    def get_discrete_scheme(self):
        if len(self.discrete_route) == 0:
            if len(self.continuous_route) == 0:
                self.calculate_continuous_scheme()
            self.calculate_discrete_scheme()
        return self.discrete_route
    
    def clear_navigation_points(self):
        self.continuous_route = []
        self.discrete_route = []
        self.navigation_points = []


def main(args=None):
    rclpy.init(args=args)
    searching = Searching()
    rclpy.spin(searching)
    searching.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
