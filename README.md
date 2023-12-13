## Instalação do pacote

No terminal, abra o diterório src dentro do workspace ROS (ros_ws) e faça o clone do pacote:
```
cd ~/ros2_ws/src && git clone git@github.com:arthur990914/turtle_control_AOM.git
```
Retorne no diretório raiz do workspace e faça o build e setup:
```
cd ~/ros2_ws && colcon build && source install/setup.bash
```

## Laboratório 5: Controle de robô uniciclo

No primeiro terminal, execute o turtlesim_node:
```
ros2 run turtlesim turtlesim_node
```

Em outro terminal, execute o nó de controle desenvolvido  para este laboratório.
```
ros2 run turtle_control_AOM turtle_control
```

Em outro terminal, envie comandos de posição x e y para a tartaruga ir:
```
ros2 topic pub /goal geometry_msgs/msg/Pose2D "{x: 7.0, y: 7.0, theta: 0.0}"
```

## Laboratório 6: Gazebo e Nav2

