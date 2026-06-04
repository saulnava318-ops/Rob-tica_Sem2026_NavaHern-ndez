import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class NodeCounter(Node):
    def __init__(self):
        super().__init__("subscriber_node")
        self.counter_= 0
        self.create_subscription(
            msg_type=Float64, 
            callback= self.sub_cbck, 
            topic= "rpm_topic", 
            qos_profile= 10)
        
        self.publisher_counter_ = self.create_publisher(
            msg_type= Float64, 
            topic='counter_topic',
            qos_profile=10
        )
        self.get_logger().info('Nodo de rad/s activo')
        
    def sub_cbck(self, msg):
        rpm = msg.data

        rads=rpm*(2.0*3.14159/60.0)

        new_msg = Float64()
        new_msg.data= rads
        self.publisher_counter_.publish(new_msg)
        self.get_logger().info(f"Recibido:{rpm:.2f} RPM -» Convertido:{rads:.2f} rad/s")

def main(args= None):
    rclpy.init(args=args)
    node = NodeCounter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '_main_':
    main()