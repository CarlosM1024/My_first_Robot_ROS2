from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class MyRobotController(Node):

    def __init__(self):
        super().__init__('my_robot_controller') #Inicializa el nodo con el nombre 'my_robot_controller'
        qos_profile = QoSProfile(depth=10) #Define un perfil de calidad de servicio con una profundidad de 10, esto significa que se almacenarán hasta 10 mensajes en la cola antes de descartarlos.

        # inicializa variables
        self.time_last = self.get_clock().now()   # almacena el tiempo actual del reloj del nodo, se utiliza para calcular el tiempo transcurrido entre las actualizaciones de odometría y posiciones articulares.
        self.odom = {'x':0.0, 'y':0.0, 'th':0.0} #diccionario para almacenar la odometría, que incluye la posición x, y y la orientación theta (th) del robot en el espacio.
        self.left_wheel_joint_pos = 0 #almacena la posición de la articulación de la rueda izquierda del robot, inicializada en 0.
        self.right_wheel_joint_pos = 0 #almacena la posición de la articulación de la rueda derecha del robot, inicializada en 0.

         # initializa publicadores y suscriptores
        self.cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile) #Suscriptor al tópico '/cmd_vel' para recibir mensajes de tipo Twist, que contienen información sobre la velocidad lineal y angular del robot. Cuando se recibe un mensaje, se llama a la función cmd_vel_callback.

        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile) #Publicador al tópico 'joint_states' para enviar mensajes de tipo JointState, que contienen información sobre las posiciones de las articulaciones del robot.
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile) #Inicializa el broadcast de transformaciones para publicar las transformaciones del robot.
        self.get_logger().info(f"{self.get_name()} started") #Imprime un mensaje en el registro indicando que el nodo ha comenzado a ejecutarse.
        

    def cmd_vel_callback(self, msg): #Función de callback que se ejecuta cada vez que se recibe un mensaje en el tópico '/cmd_vel'.
        # lee el mensaje
        vx = msg.linear.x # Velocidad lineal en el eje X, es la velocidad hacia adelante o hacia atrás del robot.
        vy = 0.0 # Velocidad lineal en el eje Y, se establece en 0.0 ya que el robot no se mueve lateralmente.
        vr = msg.angular.z # Velocidad angular alrededor del eje Z, es la velocidad de rotación del robot.
        time_now = self.get_clock().now() # Obtiene el tiempo actual del reloj del nodo para calcular el tiempo transcurrido desde la última actualización.

        # Aquí un controlador real debiera enviar órdenes a los motores
        # y recibir la posición y velocidad real desde los encoders del robot.
        # Por ahora, calculamos con los mismos datos de velocidad...

        vx_real = vx # La velocidad lineal real en el eje X, que en este caso se asume igual a la velocidad deseada (vx) ya que no hay un controlador real implementado.
        vy_real = vy # La velocidad lineal real en el eje Y, que en este caso se asume igual a la velocidad deseada (vy) ya que no hay un controlador real implementado.
        vr_real = vr # La velocidad angular real alrededor del eje Z, que en este caso se asume igual a la velocidad deseada (vr) ya que no hay un controlador real implementado.

        # actualiza la posición de las articulaciones y la odometría
        self.update_joint_positions(vx_real, vy_real, vr_real, time_now) # Actualiza las posiciones de las articulaciones del robot (ruedas) en función de las velocidades reales y el tiempo transcurrido.
        self.update_odom(vx_real, vy_real, vr_real, time_now) # Actualiza la odometría del robot (posición y orientación) en función de las velocidades reales y el tiempo transcurrido.
        x = self.odom['x']
        y = self.odom['y']
        th = self.odom['th']

        # prepara el mensaje de odometría para publicación
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.header.stamp = time_now.to_msg()
        odom_trans.transform.translation.x = x
        odom_trans.transform.translation.y = y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = \
            euler_to_quaternion(0, 0, th) # roll,pitch,yaw

        self.get_logger().info(f"x: {x}")
        self.get_logger().info(f"y: {y}")
        self.get_logger().info(f"th: {th}")

        # prepara el mensaje de posición de articulaciones, para publicación
        joint_state = JointState() # Crea un mensaje de estado de articulaciones
        joint_state.header.stamp = time_now.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_joint_pos, self.right_wheel_joint_pos]
        
        # y publica
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)


    def update_odom(self, vx, vy, vr, time_now):
        # calcula el tiempo transcurrido
        dt = (time_now.nanoseconds - self.time_last.nanoseconds) * 10**-9
        if dt > 0.1: dt = 0
        self.time_last = time_now
        # calcula desviaciones
        delta_th = vr*dt
        delta_x = (vx * cos(delta_th) - vy * sin(delta_th)) * dt
        delta_y = (vx * sin(delta_th) + vy * cos(delta_th)) * dt
        # actualiza odómetro
        self.odom['x'] += cos(self.odom['th']) * delta_x - sin(self.odom['th']) * delta_y
        self.odom['y'] += sin(self.odom['th']) * delta_x + cos(self.odom['th']) * delta_y
        self.odom['th'] += delta_th

    def update_joint_positions(self, vx, vy, vr, time_now): #Función para actualizar las posiciones de las articulaciones del robot en función de las velocidades lineales y angulares.
        L = 0.3   # distancia entre ruedas
        r = 0.05  # radio de la rueda

        # aplica cinemática inversa para estimar velocidades articulares
        l_omega = (vx - (L/2)*vr)/r # Velocidad angular de la rueda izquierda
        r_omega = (vx + (L/2)*vr)/r # Velocidad angular de la rueda derecha

        # actualiza las posiciones de las articulaciones
        dt = (time_now.nanoseconds - self.time_last.nanoseconds) * 10**-9
        self.left_wheel_joint_pos += l_omega * dt
        self.right_wheel_joint_pos += r_omega * dt


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None): #Funcion main
    try:
        rclpy.init(args=args) # Inicializa el sistema de comunicación ROS2
        my_robot_controller = MyRobotController() # Crea una instancia del controlador del robot
        rclpy.spin(my_robot_controller) # Mantiene el nodo en ejecución hasta que se reciba una señal de interrupción
    except KeyboardInterrupt: 
        print(' ... exit node')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()