import rbdl
import numpy as np


# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('/home/user/lab_ws/src/proyecto/robot_proyecto_description/urdf/robotx.urdf')
# Grados de libertad
ndof   = modelo.q_size     # Grados de libertad

# Configuracion articular
q = np.array([0.7, 0.5, -0.6, 0.3, 0, 0])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])

# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
e     = np.eye(6)               # Vector identidad

# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)

# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics
#-------------VECTOR DE GRAVEDAD--------------------
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
#---------------------------------------------------
#---------------------------------------------------
#----VECTOR DE FUERZA CENTRIFUGA Y CORIOLIS---------
rbdl.InverseDynamics(modelo, q, dq, zeros, c)
c=c-g

#----MATRIZ DE INERCIA: M[1,:] = (ID(dq,0,e[1,:])) /e[1,:]---------
for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i,:], M[i,:])
    M[i,:] = M[i,:]-g


print('Vector de gravedad')
print(np.round(g,3))
print('\n')
print('Vector de Coriolis')
print(np.round(c,3))
print('\n')
print('Matriz de Inercia')
print(np.round(M,3))
print('\n')

# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2
b2 = np.zeros(ndof)          # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia

# Parte 2: Verificacion de valores

rbdl.CompositeRigidBodyAlgorithm(modelo,q,M2)
rbdl.NonlinearEffects(modelo,q,dq,b2)
print('\n Momento de inercia por CompositeRigidBodyAlgorithm:')
print(np.round(M2,3))
print('\n Efectos no lineales')
print(np.round(b2,3))


# Parte 3: Verificacion de la expresion de la dinamica

Torque1 = M.dot(ddq) + c + g
Torque2 = M2.dot(ddq) + b2

error = Torque1 - Torque2

print('\n Torque Final (Metodo 1):')
print(np.round(Torque1,3))

print('\n Torque Final (Metodo 2):')
print(np.round(Torque1,3))

print('\n Error:')
print(np.round(error))
