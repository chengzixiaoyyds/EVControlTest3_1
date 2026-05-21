import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Bool

class PIDParameterPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pid_prameters_publisher = self.create_publisher(Float64MultiArray, 'pid_parameters', 10)
        self.pid_target_publisher = self.create_publisher(Float64, 'pid_target', 10)
        self.clear_signal_publisher = self.create_publisher(Bool, 'clear_signal', 10)

def main(args=None):
    rclpy.init(args=args)
    pid_parameter_publisher = PIDParameterPublisher('pid_parameter_publisher')
    while rclpy.ok():
        cmd = input("1. Publish PID parameters\n2. Publish PID target\n3. Clear signals\n4. Exit\nEnter your choice: ")
        if cmd == '1':
            kp = float(input("Enter Kp: "))
            ki = float(input("Enter Ki: "))
            kd = float(input("Enter Kd: "))
            msg = Float64MultiArray()
            msg.data = [kp, ki, kd]
            pid_parameter_publisher.pid_prameters_publisher.publish(msg)
        elif cmd == '2':
            msg = Float64()
            msg.data = float(input("Enter PID target: ")) 
            pid_parameter_publisher.pid_target_publisher.publish(msg)
        elif cmd == '3':
            clear_msg = Bool()
            clear_msg.data = True
            pid_parameter_publisher.clear_signal_publisher.publish(clear_msg)
        elif cmd == '4':
            break
    pid_parameter_publisher.destroy_node()
    rclpy.shutdown()
