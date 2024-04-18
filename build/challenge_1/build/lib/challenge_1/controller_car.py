import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

def calcular_distancia(punto1_x, punto1_y, punto2_x, punto2_y):
    """Calcula la distancia entre dos puntos en un plano cartesiano."""
    distancia = math.sqrt(pow((punto2_x - punto1_x), 2) + pow((punto2_y - punto1_y), 2))
    return distancia

def calcular_angulo(punto1_x, punto1_y, punto2_x, punto2_y):
    """Calcula el ángulo necesario para llegar al segundo punto desde el primero."""
    delta_x = punto2_x - punto1_x
    delta_y = punto2_y - punto1_y
    angulo_radianes = math.atan2(delta_y, delta_x)
    angulo_grados = math.degrees(angulo_radianes)

    if punto1_x == punto2_x:
        if punto1_y > punto2_y:
            angulo_grados = 90
        else:
            angulo_grados = -90
    elif punto1_y == punto2_y:
        if punto1_x > punto2_x:
            angulo_grados = 180
        else: 
            angulo_grados = 0
    elif punto2_x > punto1_x and punto2_y > punto1_y:
        angulo_grados = -angulo_grados
    elif punto2_x < punto1_x and punto2_y > punto1_y:
        angulo_grados = -angulo_grados
    elif punto2_x < punto1_x and punto2_y < punto1_y:
        angulo_grados = -angulo_grados
    else:
        angulo_grados = -angulo_grados    
    return angulo_grados

class Car_Line(Node):    
    def __init__(self):
        super().__init__('controller_node')

        while True:
            user_path_input = input("Enter an integer that represents the path you want to travel ")
            try:
                user_input_path_int = int(user_path_input)
                self.current_path = user_input_path_int
                print("You entered:", user_input_path_int)
                break
            except ValueError:
                print("Invalid input. Please enter an integer.")
        
        while True:
            user_time_input = input("Enter an integer that represents the time in seconds to complete the path ")
            try:
                user_input_time_int = int(user_time_input)
                self.user_time = user_input_time_int
                print("You entered:", user_input_time_int)
                break
            except ValueError:
                print("Invalid input. Please enter an integer.")

        self.declare_parameter('path1.wp1_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path1.wp1_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path1.wp2_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path1.wp2_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path1.wp3_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path1.wp3_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path1.wp4_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path1.wp4_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path2.wp1_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path2.wp1_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path2.wp2_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path2.wp2_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path2.wp3_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path2.wp3_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path2.wp4_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path2.wp4_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path3.wp1_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path3.wp1_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path3.wp2_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path3.wp2_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path3.wp3_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path3.wp3_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path3.wp4_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path3.wp4_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path4.wp1_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path4.wp1_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path4.wp2_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path4.wp2_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path4.wp3_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path4.wp3_y', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path4.wp4_x', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path4.wp4_y', rclpy.Parameter.Type.INTEGER)
    
        timer_period = 0.01

        self.velocidadLeft = 0.0
        self.velocidadRight = 0.0

        self.distance = 0.0
        self.angular_distance = 0.0

        self.waypoint_distance = 0.0
        self.waypoint_angle = 0.0
        self.waypoint_vel_need = 0.0
        self.waypoint_speed = 0.0

        self.flag_waypoint = 0
        self.flag_spin = 1

        x1 = 0
        y1 = 0

        x2 = 0
        y2 = 0
        x3 = 0
        y3 = 0
        x4 = 0
        y4 = 0
        x5 = 0
        y5 = 0

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Talker node successfully initialized!! :P')
        self.speed_msg = Twist()

        # Set QoS profile to match the publisher's QoS
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.sub = self.create_subscription(Float32, 'VelocityEncL', self.listener_callback_left, qos_profile)
        self.sub = self.create_subscription(Float32, 'VelocityEncR', self.listener_callback_right, qos_profile)
        self.get_logger().info('Listener Node initialized!!')

        if self.current_path == 0:
            x2 = self.get_parameter('path1.wp1_x').get_parameter_value().integer_value
            y2 = self.get_parameter('path1.wp1_y').get_parameter_value().integer_value
            x3 = self.get_parameter('path1.wp2_x').get_parameter_value().integer_value
            y3 = self.get_parameter('path1.wp2_y').get_parameter_value().integer_value
            x4 = self.get_parameter('path1.wp3_x').get_parameter_value().integer_value
            y4 = self.get_parameter('path1.wp3_y').get_parameter_value().integer_value
            x5 = self.get_parameter('path1.wp4_x').get_parameter_value().integer_value
            y5 = self.get_parameter('path1.wp4_y').get_parameter_value().integer_value
            self.get_logger().info('Parameters declared!')
        elif self.current_path == 1:
            x2 = self.get_parameter('path2.wp1_x').get_parameter_value().integer_value
            y2 = self.get_parameter('path2.wp1_y').get_parameter_value().integer_value
            x3 = self.get_parameter('path2.wp2_x').get_parameter_value().integer_value
            y3 = self.get_parameter('path2.wp2_y').get_parameter_value().integer_value
            x4 = self.get_parameter('path2.wp3_x').get_parameter_value().integer_value
            y4 = self.get_parameter('path2.wp3_y').get_parameter_value().integer_value
            x5 = self.get_parameter('path2.wp4_x').get_parameter_value().integer_value
            y5 = self.get_parameter('path2.wp4_y').get_parameter_value().integer_value
            self.get_logger().info('Parameters declared!')
        elif self.current_path == 2:
            x2 = self.get_parameter('path3.wp1_x').get_parameter_value().integer_value
            y2 = self.get_parameter('path3.wp1_y').get_parameter_value().integer_value
            x3 = self.get_parameter('path3.wp2_x').get_parameter_value().integer_value
            y3 = self.get_parameter('path3.wp2_y').get_parameter_value().integer_value
            x4 = self.get_parameter('path3.wp3_x').get_parameter_value().integer_value
            y4 = self.get_parameter('path3.wp3_y').get_parameter_value().integer_value
            x5 = self.get_parameter('path3.wp4_x').get_parameter_value().integer_value
            y5 = self.get_parameter('path3.wp4_y').get_parameter_value().integer_value
            self.get_logger().info('Parameters declared!')
        elif self.current_path == 3:
            x2 = self.get_parameter('path4.wp1_x').get_parameter_value().integer_value
            y2 = self.get_parameter('path4.wp1_y').get_parameter_value().integer_value
            x3 = self.get_parameter('path4.wp2_x').get_parameter_value().integer_value
            y3 = self.get_parameter('path4.wp2_y').get_parameter_value().integer_value
            x4 = self.get_parameter('path4.wp3_x').get_parameter_value().integer_value
            y4 = self.get_parameter('path4.wp3_y').get_parameter_value().integer_value
            x5 = self.get_parameter('path4.wp4_x').get_parameter_value().integer_value
            y5 = self.get_parameter('path4.wp4_y').get_parameter_value().integer_value
            self.get_logger().info('Parameters declared!')


        self.waypoint1_distance = calcular_distancia(x1, y1, x2, y2)
        self.waypoint2_distance = calcular_distancia(x2, y2, x3, y3)
        self.waypoint3_distance = calcular_distancia(x3, y3, x4, y4)
        self.waypoint4_distance = calcular_distancia(x4, y4, x5, y5)

        self.waypoint1_angle = calcular_angulo(x1, y1, x2, y2)
        self.waypoint2_angle = calcular_angulo(x2, y2, x3, y3)
        self.waypoint3_angle = calcular_angulo(x3, y3, x4, y4)
        self.waypoint4_angle = calcular_angulo(x4, y4, x5, y5)

        self.waypoint1_time_angle = (12/360) * abs(self.waypoint1_angle) * 2
        self.waypoint1_vel_need = self.waypoint1_distance/((self.user_time/4) - self.waypoint1_time_angle)
        self.waypoint1_speed = 0.0026 * (self.waypoint1_vel_need * self.waypoint1_vel_need) - 0.0265 * self.waypoint1_vel_need + 0.1096
        self.waypoint2_time_angle = (12/360) * abs(self.waypoint2_angle) * 2
        self.waypoint2_vel_need = self.waypoint2_distance/((self.user_time/4) - self.waypoint2_time_angle)
        self.waypoint2_speed = 0.0026 * (self.waypoint2_vel_need * self.waypoint2_vel_need) - 0.0265 * self.waypoint2_vel_need + 0.1096  
        self.waypoint3_time_angle = (12/360) * abs(self.waypoint3_angle) * 2
        self.waypoint3_vel_need = self.waypoint3_distance/((self.user_time/4) - self.waypoint3_time_angle)
        self.waypoint3_speed = 0.0026 * (self.waypoint3_vel_need * self.waypoint3_vel_need) - 0.0265 * self.waypoint3_vel_need + 0.1096  
        self.waypoint4_time_angle = (12/360) * abs(self.waypoint4_angle) * 2
        self.waypoint4_vel_need = self.waypoint4_distance/((self.user_time/4) - self.waypoint4_time_angle)
        self.waypoint4_speed = 0.0026 * (self.waypoint4_vel_need * self.waypoint4_vel_need) - 0.0265 * self.waypoint4_vel_need + 0.1096

    def timer_callback(self):

        self.speed_msg.linear.x = 0.0
        self.speed_msg.linear.y = 0.0
        self.speed_msg.linear.z = 0.0
        self.speed_msg.angular.x = 0.0
        self.speed_msg.angular.y = 0.0
        self.speed_msg.angular.z = 0.0
        
        if self.flag_waypoint == 0:
            self.waypoint_distance = self.waypoint1_distance
            self.waypoint_angle = self.waypoint1_angle
            self.waypoint_vel_need = self.waypoint1_vel_need
            self.waypoint_speed = self.waypoint1_speed
        elif self.flag_waypoint == 1:
            self.waypoint_distance = self.waypoint2_distance
            self.waypoint_angle = self.waypoint2_angle
            self.waypoint_vel_need = self.waypoint2_vel_need
            self.waypoint_speed = self.waypoint2_speed
        elif self.flag_waypoint == 2:
            self.waypoint_distance = self.waypoint3_distance
            self.waypoint_angle = self.waypoint3_angle
            self.waypoint_vel_need = self.waypoint3_vel_need
            self.waypoint_speed = self.waypoint3_speed
        elif self.flag_waypoint == 3:
            self.waypoint_distance = self.waypoint4_distance
            self.waypoint_angle = self.waypoint4_angle
            self.waypoint_vel_need = self.waypoint4_vel_need
            self.waypoint_speed = self.waypoint4_speed
        
        if self.waypoint_vel_need >= 14.9925:
            self.waypoint_speed = 0.3
            print("Velocidad maxima no alcanzable")
        if self.waypoint_vel_need <= 4.576659:
            self.waypoint_speed = 0.03
            print("Velocidad minima no alcanzable")

        waypoint_begin = self.waypoint_distance/ 4
        waypoint_mid = (self.waypoint_distance / 4) * 3
        angle = (600/360) * self.waypoint_angle
        
        # Update distance traveled
        self.distance += 0.05 * ((self.velocidadLeft + self.velocidadRight) / 2)
        self.angular_distance += 0.05 * ((self.velocidadLeft - self.velocidadRight) / 0.18)

        # Publish velocity command        
        if self.flag_waypoint < 4:

            if self.flag_spin == 1 and self.distance < self.waypoint_distance:
                if angle > 0:
                    self.speed_msg.linear.x = 0.0
                    self.speed_msg.angular.z = -0.1
                    self.publisher.publish(self.speed_msg)  
                    if self.angular_distance >= angle:
                        self.speed_msg.angular.z = 0.0
                        self.publisher.publish(self.speed_msg)
                        self.distance = 0.0
                        self.angular_distance = 0.0
                        self.flag_spin = 0
                elif angle < 0:
                    self.speed_msg.linear.x = 0.0
                    self.speed_msg.angular.z = 0.1
                    self.publisher.publish(self.speed_msg)  
                    if self.angular_distance <= angle:
                        self.speed_msg.angular.z = 0.0
                        self.publisher.publish(self.speed_msg)
                        self.distance = 0.0
                        self.angular_distance = 0.0
                        self.flag_spin = 0
                else:
                    self.flag_spin = 0 
                                    
            elif self.flag_spin == 0 and self.distance < waypoint_begin:
                self.speed_msg.linear.x = 0.1
                self.speed_msg.angular.z = 0.0
                self.publisher.publish(self.speed_msg)

            elif self.flag_spin == 0 and self.distance >= waypoint_begin and self.distance < waypoint_mid:
                self.speed_msg.linear.x = self.waypoint_speed
                self.publisher.publish(self.speed_msg)

            elif self.distance >= waypoint_mid and self.distance < self.waypoint_distance:
                self.speed_msg.linear.x = 0.1
                self.publisher.publish(self.speed_msg)   

            elif self.distance >= self.waypoint_distance:
                if angle > 0:
                    self.speed_msg.linear.x = 0.0
                    self.speed_msg.angular.z = 0.1
                    self.publisher.publish(self.speed_msg)  
                    if self.angular_distance <= -angle:
                        self.speed_msg.angular.z = 0.0
                        self.publisher.publish(self.speed_msg)
                        self.distance = 0.0
                        self.angular_distance = 0.0
                        self.flag_waypoint += 1
                        self.flag_spin = 1
                elif angle < 0:
                    self.speed_msg.linear.x = 0.0
                    self.speed_msg.angular.z = -0.1
                    self.publisher.publish(self.speed_msg)  
                    if self.angular_distance >= -angle:
                        self.speed_msg.angular.z = 0.0
                        self.publisher.publish(self.speed_msg)
                        self.distance = 0.0
                        self.angular_distance = 0.0
                        self.flag_spin = 1
                        self.flag_waypoint += 1
                else:
                    self.speed_msg.angular.z = 0.0
                    self.publisher.publish(self.speed_msg)
                    self.distance = 0.0
                    self.angular_distance = 0.0
                    self.flag_spin = 1
                    self.flag_waypoint += 1
                    self.flag_spin = 1
        
        if self.flag_waypoint == 4:
            self.speed_msg.linear.x = 0.0
            self.publisher.publish(self.speed_msg)


    def listener_callback_left(self, msg):
        self.velocidadLeft = msg.data

    def listener_callback_right(self, msg):
        self.velocidadRight = msg.data

def main(args=None):
    rclpy.init(args=args)
    m_p = Car_Line()
    rclpy.spin(m_p)
    m_p.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()