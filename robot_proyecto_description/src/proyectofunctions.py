import numpy as np
from copy import copy
import rbdl

pi = np.pi

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/robotx.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq

def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.

    """
    sth = np.sin(theta)
    cth = np.cos(theta)
    sa  = np.sin(alpha)
    ca  = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                  [sth,  ca*cth, -sa*cth, a*sth],
                  [0.0,      sa,      ca,     d],
                  [0.0,     0.0,     0.0,   1.0]])
    return T

#------------CINEMATICAS------------------------------------------

def fkine(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]

    """
    # Longitudes (en metros)

    # Matrices DH (completar)
    T1 = dh(0.116, pi/2+q[0] , 0    , pi/2)
    T2 = dh(0    , pi/2+q[1] , 0.19506, 0)
    T3 = dh(0    , q[2]      , 0.027, pi/2)
    T4 = dh(0.208, pi+q[3]   , 0    , pi/2)
    T5 = dh(0    , pi+q[4]   , 0    , pi/2)
    T6 = dh(0.059, q[5]      , 0    , 0)
    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
    return T

def ikine(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001


    q  = copy(q0)
    for i in range(max_iter):
       # Main loop
       J = jacobian_position(q)
       f = fkine(q)
       e = xdes - f[0:3,3]
       q = q + np.dot(J.T, e)
       if (np.linalg.norm(e) < epsilon):
           break
       pass
    return q


def ik_gradient(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo gradiente
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
    alpha = 0.1


    q  = copy(q0)
    for i in range(max_iter):
        # Main loop
        J = jacobian_position(q)
        f = fkine(q)
        e = xdes - f[0:3,3]
        q = q + alpha*np.dot(J.T, e)
        
        if (np.linalg.norm(e) < epsilon):
            break
          
        pass
  
    return q
    
    return q

#--------------------JACOBIANOS-------------------------------------------------

def jacobian_position(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    # Alocacion de memoria
    J = np.zeros((3,6))
    # Transformacion homogenea inicial (usando q)
    T = fkine(q)
    # Iteracion para la derivada de cada columna
    for i in range(6):
       # Copiar la configuracion articular inicial
       dq = copy(q)
       # Incrementar la articulacion i-esima usando un delta
       dq[i] = dq[i] + delta
       # Transformacion homogenea luego del incremento (q+delta)
       T_x = fkine(dq)
       # Aproximacion del Jacobiano de posicion usando diferencias finitas
       J[0, i] = (T_x[0][3] - T[0][3]) / delta
       J[1, i] = (T_x[1][3] - T[1][3]) / delta
       J[2, i] = (T_x[2][3] - T[2][3]) / delta
    return J


def jacobian_pose(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    J = np.zeros((7,6))
	# Implementar este Jacobiano aqui
    
    T = fkine(q)
    
    COOR_QUARTER = TF2xyzquat(T)
    
    for i in range(6):
    	#SE COPIA LA LA ARTICULACIÓN INICIAL}
        dq = copy(q)
        #INCREMENTO EN LA ARTICULACIÓN TENIENDO EN CUENTA LA ARTICULACIÓN [i]
        dq[i] = dq[i] + delta
    	#TRANSFORMACIÓN HOMOGENEA Y CUATERNION
        T_x = fkine(dq)
        COOR_QUARTER_X = TF2xyzquat(T_x)
    	#APROXIMACIÓN DEL JACOBIANO DE POSICION USANDO DIFERENCIAS FINITAS
    	#SON SERIES DE ITERACIONES PORQUE SE HACEN POR CADA ARTICULACION
        J[0,i] = (COOR_QUARTER_X[0] - COOR_QUARTER[0])/delta
        J[1,i] = (COOR_QUARTER_X[1] - COOR_QUARTER[1])/delta
        J[2,i] = (COOR_QUARTER_X[2] - COOR_QUARTER[2])/delta
        J[3,i] = (COOR_QUARTER_X[3] - COOR_QUARTER[3])/delta
        J[4,i] = (COOR_QUARTER_X[4] - COOR_QUARTER[4])/delta
        J[5,i] = (COOR_QUARTER_X[5] - COOR_QUARTER[5])/delta
        J[6,i] = (COOR_QUARTER_X[6] - COOR_QUARTER[6])/delta
	 
    return J



def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion

    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]

    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R
