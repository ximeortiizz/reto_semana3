import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import sys
from makbets_pose.msg import MakbetsPose

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')
        
        # Eliminamos v_max como parámetro externo para evitar cambios accidentales
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_tolerance', 0.1)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        # LÍMITES INTERNOS DE SEGURIDAD (Hardcoded para evitar reinicios)
        self._SAFE_V_MAX = 0.3  # m/s constante y conservadora
        self._SAFE_W_MAX = 0.7   # rad/s

        self.goal_pub = self.create_publisher(MakbetsPose, '/goal', 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.pose_received = False
        self.path = []
        self.current_goal_idx = 0
        self.path_completed = False

        self.generar_menu()
        self.timer = self.create_timer(0.5, self.control_rutas)

    def generar_menu(self):
        self.get_logger().info("Menu")
        print("1. 4 Puntos Manuales\n2. Triángulo\n3. Rombo\n4. Pentágono")
        opcion = input("Selección: ")
        
        if opcion == '1':
            for i in range(4):
                coords = input(f"Punto {i+1} (x y): ").split()
                self.path.append((float(coords[0]), float(coords[1])))
        elif opcion == '2': self.path = [(0.0, 0.0), (2.0, 0.0), (1.0, 1.5), (0.0, 0.0)]
        elif opcion == '3': self.path = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0), (1.0, -1.0), (0.0, 0.0)] 
        elif opcion == '4': self.path = [(0.0, 0.0), (2.0, 0.0), (2.5, 1.5), (1.0, 2.5), (-0.5, 1.5), (0.0, 0.0)]
        else: sys.exit()
        
        self.verificar_viabilidad()

    def verificar_viabilidad(self):
        for i in range(1, len(self.path)):
            dist = math.sqrt((self.path[i][0] - self.path[i-1][0])**2 + (self.path[i][1] - self.path[i-1][1])**2)
            self.get_logger().info(f"Tramo {i}: {dist:.2f}m")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.pose_received = True

    def control_rutas(self):
        if not self.pose_received or self.path_completed:
            return

        if self.current_goal_idx >= len(self.path):
            self.get_logger().info("¡Ruta completada!")
            self.path_completed = True
            return

        gx, gy = self.path[self.current_goal_idx]
        
        #calculo de distancia
        dx = gx - self.current_x
        dy = gy - self.current_y
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(f"Meta {self.current_goal_idx + 1} alcanzada.")
            self.current_goal_idx += 1
            return

        # LOGICA DE VELOCIDAD SEGURA:
        # Si estamos cerca de la meta (menos de 0.5m), reducimos la velocidad proporcionalmente
        # para evitar frenazos bruscos que causen picos de corriente.
        v_command = self._SAFE_V_MAX
        if dist_to_goal < 0.5:
            v_command = max(0.05, self._SAFE_V_MAX * (dist_to_goal / 0.5))

        msg = MakbetsPose()
        msg.x = [float(gx), float(gx)] 
        msg.y = [float(gy), float(gy)]
        msg.linear_speed = [v_command, v_command]
        msg.angular_speed = [self._SAFE_W_MAX, self._SAFE_W_MAX]
        msg.mode = 0 

        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()