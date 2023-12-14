import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import math
import random

# Nó de controle
class turtlebot_goal_node(Node):

    # Inicialização do nó
    def __init__(self):
        super().__init__('turtlebot_goal_AOM')
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    # Criando publisher
    def init_publisher(self):
        self.goal_publisher = self.create_publisher(Pose2D, '/goal', 10)

    # Criando subscribers
    def init_subscribers(self):
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
    # Inicializando variáveis utilizadas para o controle
    def init_variables(self):
        self.x = 0.0
        self.y = 0.0

        self.x_goal = 0.0
        self.y_goal = 0.0

        self.limiar_dist = 0.05

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Distância euclidiana da tartaruga até o objetivo
        dist_erro = math.sqrt((self.x_goal - self.x)**2 + (self.y_goal - self.y)**2)

        if dist_erro<self.limiar_dist:
            self.pub_callback()

    def pub_callback(self):
        goals = [Pose2D(x=0.0, y=0.0), Pose2D(x=-2.0, y=-2.0), Pose2D(x=2.0, y=-2.0), Pose2D(x=-2.0, y=2.0), Pose2D(x=2.0, y=2.0),
                 Pose2D(x=4.0, y=0.0), Pose2D(x=0.0, y=4.0), Pose2D(x=-4.0, y=0.0), Pose2D(x=0.0, y=-4.0), Pose2D(x=4.0, y=4.0)]

        goal_aleatorio = random.choice(goals)
        self.x_goal = goal_aleatorio.x
        self.y_goal = goal_aleatorio.y
        self.goal_publisher.publish(goal_aleatorio)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_goal_AOM = turtlebot_goal_node()
    rclpy.spin(turtlebot_goal_AOM)
    turtlebot_goal_AOM.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()