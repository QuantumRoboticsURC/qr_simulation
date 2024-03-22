#Program: Searching Routine
#Version: 1.1a
#Developer: @emvivas (Emiliano Vivas Rodríguez)
#Contact: a01424732@tec.mx

import rclpy, math, bisect, copy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int8, Float64
from custom_interfaces.msg import Coordinates, TargetCoordinates, NavigationCoordinates
from .submodules.alvinxy import ll2xy, xy2ll, distanceBetweenCoords

class Searching(Node):

    def __init__(self):
        super().__init__('searching')
        self.orglatitude = 19.5970212
        self.orglongitude = -99.227144
        self.state = None
        self.is_found = None
        self.is_stopped = None
        self.is_finished = None
        self.coordinates = Coordinates()
        self.point = Point()
        self.target_point = Point()
        self.scale = 7
        self.turns = 2
        self.points_distance = 150
        self.continuous_points_number = 500
        self.navigation_point_uncertainty = 10
        self.set_default_configuration()
        self.sub_state = self.create_subscription(Int8, '/state', self.state_callback, 10)
        self.sub_found = self.create_subscription(Int8, '/found', self.found_callback, 10)
        self.sub_stopped = self.create_subscription(Int8, '/stopped', self.stopped_callback, 10)
        self.sub_finished = self.create_subscription(Int8, '/finished', self.finished_callback, 10)
        self.sub_coordinates = self.create_subscription(Coordinates, '/coordinates', self.coordinates_callback, 10)
        self.pub_target_navigation_coordinates_list = self.create_publisher(NavigationCoordinates, '/target_navigation_coordinates_list', 10)
        self.pub_target_navigation_coordinates = self.create_publisher(TargetCoordinates, '/target_navigation_coordinates', 10)
        self.pub_target_navigation_coordinates_remaining_distance = self.create_publisher(Float64, '/target_navigation_coordinates_remaining_distance', 10)
        self.pub_autonomous_navigation_route_times = self.create_publisher(Int8, '/autonomous_navigation_route_times', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
    
    def set_default_configuration(self):
        self.times = Int8()
        self.continuous_route = []
        self.discrete_route = []
        self.navigation_coordinates = NavigationCoordinates()
        self.navigation_coordinates_auxiliar = None
        self.target_navigation_coordinates = None
        self.is_calculated_route = False
        self.remaining_distance = Float64()
    
    def state_callback(self,msg):
        self.state = msg.data
    
    def found_callback(self,msg):
        self.is_found = msg.data
    
    def stopped_callback(self,msg):
        self.is_stopped = msg.data

    def finished_callback(self,msg):
        self.is_finished = msg.data
    
    def coordinates_callback(self,msg):
        self.coordinates.latitude,self.coordinates.longitude = msg.latitude, msg.longitude
        self.point.x, self.point.y = ll2xy(msg.latitude, msg.longitude,self.orglatitude,self.orglongitude)

    def timer_callback(self):
        if self.is_finished == 1: #TODO
            self.set_default_configuration()
            pass
        elif self.is_stopped == 1: #TODO
            pass
        elif self.is_found == 1: #TODO
            pass
        elif self.state == 0:
            if not self.is_calculated_route:
                self.calculate_continuous_route()
                self.calculate_discrete_route()
                self.navigation_coordinates_auxiliar = copy.copy(self.navigation_coordinates.coordinates)
                self.navigation_coordinates_auxiliar.pop(0)
                self.target_navigation_coordinates = self.navigation_coordinates_auxiliar.pop(0)
                self.times.data = 0
                self.is_calculated_route = True
            elif self.target_navigation_coordinates:
                self.target_point.x, self.target_point.y = ll2xy(self.target_navigation_coordinates.latitude, self.target_navigation_coordinates.longitude, self.orglatitude, self.orglongitude)
                self.remaining_distance.data = distanceBetweenCoords(self.coordinates.latitude, self.coordinates.longitude, self.target_navigation_coordinates.latitude, self.target_navigation_coordinates.longitude)
                if self.point.x >= self.target_point.x - self.navigation_point_uncertainty and self.point.x <= self.target_point.x + self.navigation_point_uncertainty and self.point.y >= self.target_point.y - self.navigation_point_uncertainty and self.point.y <= self.target_point.y + self.navigation_point_uncertainty:
                    self.target_navigation_coordinates = self.navigation_coordinates_auxiliar.pop(0) if len(self.navigation_coordinates_auxiliar)>0 else None
            elif self.is_calculated_route:
                self.navigation_coordinates_auxiliar = copy.copy(self.navigation_coordinates.coordinates)
                self.target_navigation_coordinates = self.navigation_coordinates_auxiliar.pop(0)
                self.times.data += 1
        self.pub_target_navigation_coordinates_list.publish(self.navigation_coordinates)
        self.pub_target_navigation_coordinates.publish(self.target_navigation_coordinates if self.target_navigation_coordinates else TargetCoordinates())
        self.pub_target_navigation_coordinates_remaining_distance.publish(self.remaining_distance)
        self.pub_autonomous_navigation_route_times.publish(self.times)

    def calculate_continuous_route(self):
        self.continuous_route = []
        angle_max = 2 * math.pi * self.turns
        points_number = int(angle_max * self.continuous_points_number)
        delta_angle = angle_max / points_number
        for index in range(points_number):
            angle = index * delta_angle
            self.continuous_route.append((int(self.scale * angle * math.cos(angle) + self.point.x), int(self.scale * angle * math.sin(angle) + self.point.y)))
        
    def calculate_discrete_route(self):
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
            target_coordinates = TargetCoordinates()
            target_coordinates.latitude,target_coordinates.longitude = xy2ll(current_point[0],current_point[1],self.orglatitude,self.orglongitude)
            self.navigation_coordinates.coordinates.append(target_coordinates)

def main(args=None):
    rclpy.init(args=args)
    searching = Searching()
    rclpy.spin(searching)
    searching.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
