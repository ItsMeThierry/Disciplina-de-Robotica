#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

def euler_from_quaternion(quaternion):
    """Converte quaterniões da odometria para ângulos de Euler (yaw)"""
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        
        # Declaração dos parâmetros carregados do YAML
        self.declare_parameter('A', 1.0)
        self.declare_parameter('B', 2.0)
        self.declare_parameter('omega', 0.5)
        self.declare_parameter('K_x', 1.5)
        self.declare_parameter('K_y', 1.5)
        self.declare_parameter('K_theta', 2.0)
        self.declare_parameter('K_ff', 1.0)
        self.declare_parameter('is_open_loop', False)
        
        # Leitura dos parâmetros
        self.A = self.get_parameter('A').value
        self.B = self.get_parameter('B').value
        self.omega = self.get_parameter('omega').value
        self.K_x = self.get_parameter('K_x').value
        self.K_y = self.get_parameter('K_y').value
        self.K_theta = self.get_parameter('K_theta').value
        self.K_ff = self.get_parameter('K_ff').value
        self.is_open_loop = self.get_parameter('is_open_loop').value
        
        # Variáveis de pose atual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Publicador e Subscritor
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        
        # Ciclo de controlo a 50Hz (0.02s)
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info(f"Nó de trajetória iniciado. Malha Aberta: {self.is_open_loop}")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = euler_from_quaternion(msg.pose.pose.orientation)

    def control_loop(self):
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        # Cinemática Direta: Posição Desejada
        xd = self.A * math.sin(self.omega * t)
        yd = self.B * math.sin(2 * self.omega * t)
        
        # Primeira Derivada (Velocidade)
        xd_dot = self.A * self.omega * math.cos(self.omega * t)
        yd_dot = 2 * self.B * self.omega * math.cos(2 * self.omega * t)
        
        # Segunda Derivada (Aceleração para cálculo do omega)
        xd_ddot = -self.A * (self.omega**2) * math.sin(self.omega * t)
        yd_ddot = -4 * self.B * (self.omega**2) * math.sin(2 * self.omega * t)
        
        # Pose de Referência
        theta_d = math.atan2(yd_dot, xd_dot)
        
        # Velocidades Feedforward (Malha Aberta)
        vd_f = math.sqrt(xd_dot**2 + yd_dot**2)
        denom = xd_dot**2 + yd_dot**2
        omega_d_f = (xd_dot * yd_ddot - yd_dot * xd_ddot) / denom if denom > 1e-6 else 0.0
        
        cmd = Twist()
        
        if self.is_open_loop:
            # Aplica diretamente as referências sem olhar aos sensores
            cmd.linear.x = float(vd_f)
            cmd.angular.z = float(omega_d_f)
        else:
            # Erro Global
            ex = xd - self.x
            ey = yd - self.y
            
            # Erro no Referencial do Robô (Matriz de Rotação Inversa)
            ex_b = math.cos(self.theta) * ex + math.sin(self.theta) * ey
            ey_b = -math.sin(self.theta) * ex + math.cos(self.theta) * ey
            
            # Normalização do erro angular para o caminho mais curto (-pi a pi)
            e_theta = math.atan2(math.sin(theta_d - self.theta), math.cos(theta_d - self.theta))
            
            # Lei de Controle (Feedforward + Proporcional)
            v = (self.K_ff * vd_f) + (self.K_x * ex_b)
            w = (self.K_ff * omega_d_f) + (self.K_y * ey_b) + (self.K_theta * e_theta)
            
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()