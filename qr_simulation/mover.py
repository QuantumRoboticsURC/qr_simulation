import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard


class Mover(Node):
    def __init__(self):
        super().__init__("Mover")
        self.pub = self.create_publisher(Twist,"/cmd_vel",10)
        self.twist = Twist()
        self.main = self.create_timer(0.01,self.main)
        
    def main(self):
            
        listener = keyboard.Listener(on_press=self.on_press,on_release=self.on_key_release)
        listener.start()  # start to listen on a separate thread
        listener.join() 

    def on_key_release(self,key):
        self.twist.linear.x=0.0
        self.twist.linear.y=0.0
        self.twist.angular.z=0.0
        self.pub.publish(self.twist)
    
    def on_press(self,key):
        if key==keyboard.Key.esc:
            return False
        try: 
            k = key.char   
        except:
            k = key.name 
        if k=="w":
            self.twist.linear.x=0.5
            self.twist.linear.y=0.0
            self.twist.angular.z=0.0
        elif k=="s":
            self.twist.linear.x=-0.5
            self.twist.linear.y=0.0
            self.twist.angular.z= 0.0
        elif k=="a":
            self.twist.linear.x=0.0
            self.twist.linear.y=-0.5
            self.twist.angular.z=0.0
        elif k=="d":
            self.twist.linear.x=0.0
            self.twist.linear.y=0.5
            self.twist.angular.z=0.0 
        elif k=="left":
            self.twist.linear.x=0.0
            self.twist.linear.y=0.0
            self.twist.angular.z=-0.1
        elif k=="right":
            self.twist.linear.x=0.0
            self.twist.linear.y=0.0
            self.twist.angular.z=0.1 
        else:
            self.twist.linear.x=0.0
            self.twist.linear.y=0.0
            self.twist.angular.z=0.0
        self.pub.publish(self.twist)
  
def main(args=None):
    rclpy.init(args=args)
    m = Mover()
    rclpy.spin(m)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()