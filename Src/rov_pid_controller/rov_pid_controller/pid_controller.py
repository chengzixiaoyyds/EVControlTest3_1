import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Bool

dt = 0.01  # 控制循环时间间隔，单位秒

class ROVModel:
    """水下机器人物理模型"""
    def __init__(self):
        self.depth = 0.0
        self.v = 0.0
        self.a = 0.0
        self.m = 10.0
        self.kv = 1.0
        self.f = 1.0
        
    def reset(self):
        self.depth = 0.0
        self.v = 0.0
        self.a = 0.0

    def step(self, F, dt):
        self.a = (F - self.kv * self.v - self.f) / self.m
        self.v += self.a * dt
        self.depth += self.v * dt
        if self.depth < 0.0:
            self.depth = 0.0
            self.v = 0.0
        return self.depth
    
class PIDController:
    """增量式PID控制器
    
    增量式公式:
        Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
        u(k) = u(k-1) + Δu(k)
    """
    def __init__(self, Kp, Ki, Kd, target):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        self.prev_error = 0.0      # e(k-1)
        self.prev_prev_error = 0.0 # e(k-2)
        self.output = 0.0          # u(k-1)
        
    def reset(self):
        self.prev_error = 0.0
        self.prev_prev_error = 0.0
        self.output = 0.0

    def update(self, measurement, dt):

        error = self.target - measurement  # e(k)

        delta_u = (
            self.Kp * (error - self.prev_error) +
            self.Ki * error * dt +
            self.Kd * (error - 2.0 * self.prev_error + self.prev_prev_error) / dt
        )

        self.output += delta_u
        self.prev_prev_error = self.prev_error
        self.prev_error = error

        return self.output

    def set_params(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd

    def set_target(self, target):
        self.target = target

class PIDControllerNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.controller = PIDController(0.0, 0.0, 0.0, 20.0)
        self.model = ROVModel()
        self.create_subscription(Float64MultiArray, 'pid_parameters', self.pid_parameters_callback, 10)
        self.create_subscription(Float64, 'pid_target', self.pid_target_callback, 10)
        self.create_subscription(Bool, 'clear_signal', self.clear_signal_callback, 10)
        self.depth_publisher = self.create_publisher(Float64, 'pid_depth_output', 10)
        self.timer = self.create_timer(dt, self.control_loop)
    def pid_parameters_callback(self, msg):
        Kp, Ki, Kd = msg.data
        self.controller.set_params(Kp, Ki, Kd)
        self.controller.reset()

    def pid_target_callback(self, msg):
        target = msg.data
        self.controller.set_target(target)

    def clear_signal_callback(self, msg):
        if msg.data:
            self.controller.reset()
            self.model.reset()

    def control_loop(self):
        depth = self.model.depth
        depth_msg = Float64()
        depth_msg.data = depth
        self.depth_publisher.publish(depth_msg)
        control_output = self.controller.update(depth, dt)
        self.model.step(control_output, dt)

def main(args=None):
    rclpy.init(args=args)
    pid_controller_node = PIDControllerNode('pid_controller_node')
    rclpy.spin(pid_controller_node)
    pid_controller_node.destroy_node()
    rclpy.shutdown()
