import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8,Bool,Float64
from geometry_msgs.msg import Twist
from .submodules.alvinxy import *

class Main(Node):
    def __init__(self):
        self.subscribe = self.create_subscription(Twsit,"cmd_vel",self.update_velocity)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.angle = 0.0
        self.refresh_rate = 0.01
        self.orglat = 19.5970212
        self.orglong = -99.227144
        
    
    def update_velocity(self,msg):
        self.linear_velocity = msg.linear.x
        self.angle += msg.angular.z*self.refresh_rate 
        self.angle = (self.angle+2*math.pi)%2*math.pi
        
        
def main(args=None):
    rclpy.init(args=args)
    m = Main()
    rclpy.spin(m)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
        
        
        
        
    