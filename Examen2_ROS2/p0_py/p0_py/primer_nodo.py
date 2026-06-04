import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from math import sin

class MyNodo(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(
            msg_type = Float64,
            topic = "rpm_topic", 
            qos_profile = 10)
        
        self.amplitude = 0.50
        self.frequency = 1.0
        self.time_counter = 0.0

        self.timer_period = 0.1

        self.timer_ = self.create_timer(
            self.timer_period, callback = self.cbck)
        self.get_logger().info('Nodo de RPM con seno activado')
        
    def cbck(self):
        msg = Float64()

        val=self.amplitude*sin(2*3.1416*self.frequency*self.time_counter)+0.50

        msg.data = val
        self.publisher_.publish(msg)
        self.time_counter += self.timer_period

def main(args= None):
    rclpy.init(args=args)
    node = MyNodo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== '_main_':
    main()