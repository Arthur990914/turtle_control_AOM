import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
import math

def quaternion2euler(quaternion):
    t0 = +2.0 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2])
    t1 = +1.0 - 2.0 * (quaternion[0]**2 + quaternion[1]**2)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0])
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
    t4 = +1.0 - 2.0 * (quaternion[1]**2 + quaternion[2]**2)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

# Nó de controle
class turtlebot_control_node(Node):

    # Inicialização do nó
    def __init__(self):
        super().__init__('turtlebot_control_AOM')
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    # Criando publisher
    def init_publisher(self):
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    # Criando subscribers
    def init_subscribers(self):
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_subscriber = self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)
        
    # Inicializando variáveis utilizadas para o controle
    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.theta= 0.0

        self.x_goal = 0.0
        self.y_goal = 0.0

        self.limiar_dist = 0.05
        self.limiar_theta = 0.2

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientacao_quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        
        roll,pitch,yaw = quaternion2euler(orientacao_quaternion)

        self.theta = yaw
        self.pub_callback()

    def goal_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y  

    def pub_callback(self):
        twist_msg = Twist()
        
        #  Ângulo da tartaruga até o objetivo
        theta_goal = math.atan2(self.y_goal-self.y, self.x_goal-self.x)

        # Diferença do angulo atual da tartaruga em relação ao angulo para o objetivo, entre -pi e pi radianos
        theta_erro = theta_goal - self.theta

        if theta_erro > math.pi:
            theta_erro -= 2 * math.pi
        elif theta_erro < -math.pi:
            theta_erro += 2 * math.pi

        # Distância euclidiana da tartaruga até o objetivo
        dist_erro = math.sqrt((self.x_goal - self.x)**2 + (self.y_goal - self.y)**2)

        # Se a distância da tartaruga até o objetivo for maior que o limiar, 
        # verifica a diferença do angulo da tartaruga para o objetivo.
        # Se ela estiver apontando para o objetivo (se -limiar_theta<theta_erro<limiar_theta),
        # ela avança para frente até chegar, caso contrario ajusta o angulo.
        if dist_erro > self.limiar_dist:
            if theta_erro > self.limiar_theta:
                twist_msg.angular.z = 0.5
            elif theta_erro < -self.limiar_theta:
                twist_msg.angular.z = -0.5
            else:
                twist_msg.linear.x = 0.5
        else:
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0

        self.velocity_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    turtlebot_control_AOM = turtlebot_control_node()
    rclpy.spin(turtlebot_control_AOM)
    turtlebot_control_AOM.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()