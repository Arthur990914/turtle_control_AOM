import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose as TurtlePose
import math

# Nó de controle
class turtle_control_node(Node):

    # Inicialização do nó
    def __init__(self):
        super().__init__('turtle_control_AOM')
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    # Criando publisher
    def init_publisher(self):
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    # Criando subscribers
    def init_subscribers(self):
        self.pose_subscriber = self.create_subscription(TurtlePose, '/turtle1/pose', self.pose_callback, 10)
        self.goal_subscriber = self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)
        
    # Inicializando variáveis utilizadas para o controle
    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.theta= 0.0

        self.x_goal = 5.54  # (5.54;5,54) é a ponto inicial da tartaruga
        self.y_goal = 5.54

        self.limiar_dist = 0.05
        self.limiar_theta = 0.01

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.pub_callback()

    def goal_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y  

    def pub_callback(self):
        twist_msg = Twist()
        
        #  Ângulo da tartaruga até o objetivo
        theta_goal = math.atan2(self.y_goal-self.y, self.x_goal-self.x)

        # Diferença do angulo atual da tartaruga em relação ao angulo para o objetivo
        theta_erro = theta_goal - self.theta

        # Distância euclidiana da tartaruga até o objetivo
        dist_erro = math.sqrt((self.x_goal - self.x)**2 + (self.y_goal - self.y)**2)

        # Se a distância da tartaruga até o objetivo for maior que o limiar, 
        # verifica a a diferença do angulo da tartaruga para o objetivo.
        # Se ela estiver apontando para o objetivo (se -limiar_theta<theta_erro<limiar_theta),
        # ela avança para frente até chegar, caso contrario ajusta o angulo.
        if dist_erro > self.limiar_dist:
            if theta_erro > self.limiar_theta:
                twist_msg.angular.z = 1.0
            elif theta_erro < -self.limiar_theta:
                twist_msg.angular.z = -1.0
            else:
                twist_msg.linear.x = 1.0
        else:
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0

        self.velocity_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    turtle_control_AOM = turtle_control_node()
    rclpy.spin(turtle_control_AOM)
    turtle_control_AOM.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()