#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from markers import *
from proyectofunctions import *
from roslib import packages
import rbdl


rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual  = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])
# Archivos donde se almacenara los datos
fqact = open("/home/user/lab_ws/qactual.txt", "w")
fqdes = open("/home/user/lab_ws/qdeseado.txt", "w")
fxact = open("/home/user/lab_ws/xactual.txt", "w")
fxdes = open("/home/user/lab_ws/xdeseado.txt", "w")

# Nombres de las articulaciones
jnames = ['revo_1', 'revo_2', 'revo_3','revo_4', 'revo_5', 'revo_6']

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# =============================================================
# Configuracion articular inicial (en radianes)
q = np.array([0.0, 0., 0., 0., 0., 0.0])
# Velocidad inicial
dq = np.array([0., 0., 0., 0., 0., 0.])
# Aceleracion inicial
ddq = np.array([0., 0., 0., 0., 0., 0.])
# Configuracion articular deseada
qdes = np.array([1.3, 0.6, 0.3, 0.0, 0.0, 0.0])
# Velocidad articular deseada
dqdes = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
# Aceleracion articular deseada
ddqdes = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])
# =============================================================

# Posicion resultante de la configuracion articular deseada
xdes = fkine(qdes)[0:3,3]
# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('../urdf/robotx.urdf')
ndof   = modelo.q_size     # Grados de libertad
zeros = np.zeros(ndof)     # Vector de ceros

# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)

# Bucle de ejecucion continua
t = 0.0

# Se definen las ganancias del controlador
valores = 0.1*np.array([10.0, 10.0, 10.0])
Kp = np.diag(valores)
Kp1=Kp[0:3,0:3]
Kd = 2*np.sqrt(Kp)
kd1=Kd[0:3,0:3]
# delta=0.0001
JAant = np.zeros(ndof) 

while not rospy.is_shutdown():

    # Leer valores del simulador
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Posicion actual del efector final
    x = fkine(q)[0:3,3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')

    # ----------------------------
    # Control dinamico (COMPLETAR)
    # ----------------------------
    b2 = np.zeros(ndof)          # Para efectos no lineales
    M2 = np.zeros([ndof, ndof])  # Para matriz de inercia

    rbdl.NonlinearEffects(modelo,q,dq,b2)
    rbdl.CompositeRigidBodyAlgorithm(modelo, q, M2,update_kinematics = True  )
    JA = jacobian_position(q,dt) 

    JAD = (JA - JAant)/dt

    dx = JA@dq


    de = -dx

    e = xdes-x


    # print(np.shape(np.linalg.pinv(JA))) #6x6
    # print(np.shape((-JAD@dq+kd1@de+Kp1@e))) #6x6
    # print(np.shape(M2@ddq)) #6x6
    # print(np.shape(b2)) #6x6
    
    e_n = np.linalg.norm(e)

    epsilon = 0.001
    
    if e_n < epsilon:
        break

    u=M2@np.linalg.pinv(JA)@(-JAD@dq+kd1@de+Kp1@e)+b2

    print(u)
    JAant = JA

    # Simulacion del robot
    robot.send_command(u)

    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

fqact.close()
fqdes.close()
fxact.close()
fxdes.close()
