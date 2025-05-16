#!/usr/bin/env python3
import rospy
import math
import time
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, TeleportAbsolute

class MultiTurtleEvaluator:
    def __init__(self):
        rospy.init_node('multi_turtle_evaluator')

        # 1) Configurar entorno: matar y (re)spawn de tortugas
        rospy.wait_for_service('/kill')
        rospy.wait_for_service('/spawn')
        rospy.wait_for_service('/turtle1/teleport_absolute')

        kill = rospy.ServiceProxy('/kill', Kill)
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

        # Matar la turtle1 por si acaso y volverla a crear en el centro
        try:
            kill('turtle1')
        except:
            pass
        spawn(5.5, 5.5, 0.0, 'turtle1')
        teleport(5.5, 5.5, 0.0)

        # Definir tortugas destino
        self.targets = {
            'turtle2': (2.0, 2.0),
            'turtle3': (8.0, 2.0),
            'turtle4': (8.0, 8.0),
            'turtle5': (2.0, 8.0),
        }
        # Spawn de cada tortuga destino
        for name, (x,y) in self.targets.items():
            spawn(x, y, 0.0, name)

        # Umbral de seguridad (m)
        self.threshold = 0.5

        # Variables de estado
        self.pose = None
        self.start_time = None
        self.max_deviation = 0.0
        self.unsafe_count = 0
        self._in_unsafe = False

        # ROS I/O
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/turtle1/pose', Pose, self.cb_pose)

        # Esperar la primera pose
        rospy.loginfo("Esperando pose inicial de turtle1...")
        while not rospy.is_shutdown() and self.pose is None:
            rospy.sleep(0.1)

    def cb_pose(self, msg):
        self.pose = msg

    def line_deviation(self, p, a, b):
        """Distancia punto–línea AB"""
        x0,y0 = p
        x1,y1 = a
        x2,y2 = b
        num = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)
        den = math.hypot(y2-y1, x2-x1)
        return num/den if den>0 else 0.0

    def navigate_to(self, goal_xy):
        """Navega turtle1 hacia goal_xy midiendo métricas."""
        rate = rospy.Rate(20)
        # reset inseguridad por tramo
        self._in_unsafe = False

        while not rospy.is_shutdown():
            dx = goal_xy[0] - self.pose.x
            dy = goal_xy[1] - self.pose.y
            dist = math.hypot(dx, dy)
            if dist < 0.1:
                break

            # controlador P
            angle_to_goal = math.atan2(dy, dx)
            dtheta = math.atan2(math.sin(angle_to_goal - self.pose.theta),
                                math.cos(angle_to_goal - self.pose.theta))

            twist = Twist()
            twist.linear.x = min(1.5 * dist, 2.0)
            twist.angular.z = 4.0 * dtheta
            self.pub.publish(twist)

            # métrica desviación respecto a la línea actual
            # línea entre el último punto y este objetivo
            dev = self.line_deviation(
                (self.pose.x, self.pose.y),
                self.last_point,
                goal_xy
            )
            if dev > self.max_deviation:
                self.max_deviation = dev

            # evento de transición insegura
            if dev > self.threshold and not self._in_unsafe:
                self.unsafe_count += 1
                self._in_unsafe = True
            elif dev <= self.threshold and self._in_unsafe:
                self._in_unsafe = False

            rate.sleep()

        # detener al llegar
        self.pub.publish(Twist())
        rospy.sleep(0.3)

    def run(self):
        rospy.sleep(1.0)
        self.start_time = time.time()
        # Convertir dict a lista ordenada de tuplas (x,y)
        sequence = list(self.targets.values())

        # Empezar en la posición actual
        self.last_point = (self.pose.x, self.pose.y)

        for pt in sequence:
            rospy.loginfo(f"Navegando a {pt} …")
            self.navigate_to(pt)
            self.last_point = pt

        # volver al inicio
        rospy.loginfo("Regresando al punto de partida …")
        start_xy = (5.5, 5.5)
        self.navigate_to(start_xy)

        total_time = time.time() - self.start_time
        rospy.loginfo("=== EVALUACIÓN COMPLETA ===")
        rospy.loginfo(f"Tiempo total: {total_time:.2f} s")
        rospy.loginfo(f"Desviación máxima: {self.max_deviation:.2f} m")
        rospy.loginfo(f"Transiciones inseguras: {self.unsafe_count}")

if __name__ == '__main__':
    try:
        node = MultiTurtleEvaluator()
        node.run()
    except rospy.ROSInterruptException:
        pass
