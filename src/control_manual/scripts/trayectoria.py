#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from nav_msgs.msg import Odometry 
import numpy as np

from lyapunov import *
import matplotlib.pyplot as plt


class Bebop_functions():
    def __init__(self):
        # Bebop_control node creation
        rospy.init_node('Bebop_control', anonymous=True)

        # Publisher para despegue
        self.takeoff_publisher = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)

        # Publisher para aterrizaje
        self.landing_publisher = rospy.Publisher('/bebop/land', Empty, queue_size=10)

        # Publisher comando de velocidad
        self.cmdvel_publisher = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/bebop/states/ardrone3/PilotingState/FlyingStateChanged'. self.update_state is called
        # when a message of type Ardrone3PilotingStateFlyingStateChanged is received.
        self.pilotingstate_subscriber = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/FlyingStateChanged',
                                                         Ardrone3PilotingStateFlyingStateChanged, self.update_state)

        self.bebopcoord_subscriber = rospy.Subscriber("/bebop/odom", Odometry, self.bebop_pose)

        # self.bebopvel_subscriber = rospy.Subscriber('bebop/states/ARDrone3/PilotingState/SpeedChanged', Ardrone3PilotingStateSpeedChanged)

        self.state = None
        self.rate = rospy.Rate(10)



    def takeoff(self):  # Take off command function
        while self.state != 2 and not rospy.is_shutdown():
            hello_str = "Takeoff command has been sent %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.takeoff_publisher.publish(Empty())
            self.rate.sleep()
        print("Takeoff is done!")


    def land(self):  # Land command function
        self.finished = True
        self.cmdvel_publisher.publish(Twist())
        while self.state != 0 and not rospy.is_shutdown():
            hello_st = "Land command has been sent %s" % rospy.get_time()
            rospy.loginfo(hello_st)
            self.landing_publisher.publish(Empty())
            self.rate.sleep()
        print("Landing completed succesfully!")



    # /bebop/states... callback subscriber to read the state of the parrot
    def update_state(self, message):
        self.state = message.state


    def safety_check(self):
        if self.bebopose.position.x > 8.0 or self.bebopose.position.y > 8.0 or self.bebopose.position.z > 8.0:
            print "EMERGENCY LANDING"
            self.finished = True
            self.finished_yaw = True

   
    def bebop_pose(self, msg):

        # self.bebopose = msg.transform.translation
        # quat = msg.transform.rotation
        self.bebopose = msg.pose.pose
        quat = self.bebopose.orientation
        (roll, pitch, self.yaw) = tf.transformations.euler_from_quaternion(
            (quat.x, quat.y, quat.z, quat.w))
        self.safety_check()


    def lyapunov(self):

        #################### TIEMPO ###################
        tf = 40 # tiempo de simulacion
        ts = 0.5 #  tiempo de muestreo
        t = np.arange(0,tf+ts,ts) # vector tiempo
        N = len(t) # cantidad de muestras

        ###################  PARAMETROS DEL ROBOT ################### 
        # a = 0.3  # Altura del robot en metros [m]
        # b = 0.1 # Distancia hacia el punto de control en metros [m]

        ################### CONDICIONES INICIALES ###################
        # Centro de masa
        x1 = np.zeros(N+1) # Inicializar variables 
        y1 = np.zeros(N+1)
        z1 = np.zeros(N+1)


        x1[0] =  0   # Posicion inicial en el eje x en metros [m]
        y1[0] =  0  # Posicion inicial en el eje y en metros [m]
        z1[0] =  0.05   # Posicion inicial en el eje z en metros [m]


        phi = np.zeros(N+1)
        thetha = np.zeros(N+1)
        psi = np.zeros(N+1)

        psi[0] = 0*(np.pi/180)  # Orientacion inicial en radianes [rad]



        # Punto de control
        hx = np.zeros(N+1) # Inicializar variables 
        hy = np.zeros(N+1)
        hz = np.zeros(N+1)

        # Cinematica directa
        hx[0] = x1[0] #+b*np.cos(psi[0])
        hy[0] = y1[0] #+b*np.sin(psi[0])
        hz[0] = z1[0] #+a

        ######################### POSICION DESEADA ##################
        # hxd = 3
        # hyd = -2.5
        # hzd = 3
        psid = -90*(np.pi/180) * np.ones(N)
        
        """curva"""
        # hxd = 2 * np.cos(0.1 * t)
        # hyd = 0.2 * t
        # hzd = 3 * np.ones(N)

        """cuadrado"""
        # div = 500
        div = round(N/4)
        pointX = [0, 0, 2,  2, 0]
        pointY = [0, -2, -2, 0, 0, 2]
        # pointX = [-2,-1,1,2,2.1,1,-1,-2,-2]
        # pointY = [0.5,1,1,0.5,-0.5,-1,-1,-0.5,0.5]
        pointZ = 1*np.ones(len(pointX))

        px = []
        py = []
        pz = []

    
        for p in range(len(pointX)-1):
            px.append(np.linspace(pointX[p],pointX[p+1],div))
            py.append(np.linspace(pointY[p],pointY[p+1],div))
            pz.append(np.linspace(pointZ[p],pointZ[p+1],div))

        hxd = np.hstack(px)
        hyd = np.hstack(py)
        hzd = np.hstack(pz)

        sizePoints = len(hxd)



        ################### VELOCIDADES DE REFERENCIA #################### 

        ufRef = np.zeros(N) # Velocidad lineal en metros/segundos [m/s] eje x 
        ulRef = np.zeros(N) # Velocidad lineal en metros/segundos [m/s] eje y
        uzRef = np.zeros(N) # Velocidad lineal en metros/segundos [m/s] eje z
        wRef = np.zeros(N) # Velocidad angular en radianes/segundos [rad/s]

        ################### ERRORES ####################
        hxe = np.zeros(N) # Error en el eje x en metros [m]
        hye = np.zeros(N)  # Error en el eje y en metros [m]
        hze = np.zeros(N) # Error en el eje z en metros [m]
        psie = np.zeros(N) # Error de orientacion en radianes [rad]


        ################### BUCLE ####################
        for k in range(N-1):

        ###################### CONTROLADOR ######################
            start_time = rospy.Time.now() # Tiempo actual

            # self.bebop_pose()
          

            # Errores

            # print(self.bebopose.position.x)

            # Punto mas cercano
            # minimo = 1000

            """+++++++++++++++++++++++++++++++++++++++++++"""
     
            # for p in range(sizePoints):
            #     distancia = np.sqrt((hxd[p]-self.bebopose.position.x)**2 + (hyd[p]-self.bebopose.position.y)**2 + (hzd[p]-self.bebopose.position.z)**2)
            #     print(distancia)
            #     if distancia < minimo:
            #         minimo = distancia
            #         pos = p
            """+++++++++++++++++++++++++++++++++++++++++++++"""
                    

            hxe[k]  =  hxd[k] - self.bebopose.position.x
            hye[k]  =  hyd[k] - self.bebopose.position.y
            hze[k]  =  hzd[k] - self.bebopose.position.z
            psie[k] =  psid[k]  - self.yaw

            # print(self.bebopose.position.z)
            print(self.yaw)

            he = np.array([[hxe[k]],[hye[k]],[hze[k]],[psie[k]]]) # vector de errores (4x1)

            # Matriz Jacobiana

            # J = np.array([[ np.cos(psi[k]), -np.sin(psi[k]), 0  , 0],
            #               [ np.sin(psi[k]), np.cos(psi[k]) , 0  , 0],
            #               [0              , 0              , 1  , 0],
            #               [0              , 0              , 0  , 1]])

            J = np.array([[ np.cos(self.yaw), -np.sin(self.yaw), 0  , 0],
                          [ np.sin(self.yaw), np.cos(self.yaw) , 0  , 0],
                          [0              , 0              , 1  , 0],
                          [0              , 0              , 0  , 1]])

            # Parametros de control

            K = 3 * np.array([[1,0,0,0],
                              [0,1,0,0],
                              [0,0,1,0],
                              [0,0,0,1]])

            # Ley de control

            qpRef = np.linalg.pinv(J).dot(K).dot(he) # velocidades de referencia

            ufRef[k] = qpRef[0][0]
            ulRef[k] = qpRef[1][0]
            uzRef[k] = qpRef[2][0]
            wRef[k] = qpRef[3][0]
          
            twist = Twist()
            twist.linear.x = ufRef[k]
            twist.linear.y = ulRef[k]
            twist.linear.z = uzRef[k]
            twist.angular.z = wRef[k]
            self.cmdvel_publisher.publish(twist)


            #################### APLICAR ACCIONES DE CONTROL #####################

            # # Integral numerica
            # psi[k+1] = psi[k]+ts*wRef[k]

            # # Modelo cinematico
            # x1p = ufRef[k]*np.cos(psi[k+1])-ulRef[k]*np.sin(psi[k+1])
            # y1p = ufRef[k]*np.sin(psi[k+1])+ulRef[k]*np.cos(psi[k+1])
            # z1p = uzRef[k]

            # # Integral numeric
            # x1[k+1] = x1[k] + ts*x1p
            # y1[k+1] = y1[k] + ts*y1p
            # z1[k+1] = z1[k] + ts*z1p

            # # Cinematica directa
            # hx[k+1] = x1[k+1]+b*np.cos(psi[k+1])  
            # hy[k+1] = y1[k+1]+b*np.sin(psi[k+1])
            # hz[k+1] = z1[k+1]+a


            elapse_time = (rospy.Time.now() - start_time).to_sec()
            rospy.sleep(ts-elapse_time)
            # rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        node = Bebop_functions()
        # node.waypoint()
        
        node.takeoff()
        rospy.sleep(2)
        node.lyapunov()
        rospy.sleep(2)
      
        node.land()  
    except rospy.ROSInterruptException as e:
        print "Exception: {0}".format(str(e))