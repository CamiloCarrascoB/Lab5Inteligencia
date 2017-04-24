#!/usr/bin/env python
# Autor> Ruben Claveria Vega
# Curso> EL5206 Laboratorio de Inteligencia Computacional y Robotica

# En lugar de 'testpy', deberia ir el nombre del paquete creado.
import roslib; roslib.load_manifest('EL5206')
import robot_utilities
import time
import rospy
import math
import numpy
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from std_msgs.msg import String 
import matplotlib.pyplot as plt
import random

class Controller:
	def __init__(self):

		# inicializa nodo ROS
		rospy.init_node('Simulation');

		# robotID = ID del robot que se desea controlar
		# si hay solo un robot, robotID debe ser ""
		# si hay mas de un robot, robotID debe ser de la forma "/robot_0", "/robot_1", etc.
		robotID = ""

		# Posicion y orientacion iniciales.
		# IMPORTANTE: deben ser consistentes con la del archivo .world
		init_x = -2.0
		init_y = -2.0
		init_angle = 90.0

		# creacion de un objeto de tipo Robot
		R = robot_utilities.Robot(robotID, init_x, init_y, init_angle)

		# Subscripcion a los topicos de interes: odometria (Odometry) y sensor de distancia (LaserScan)
		rospy.Subscriber(robotID+'/odom', Odometry, R.odom_callback)
		rospy.Subscriber(robotID+'/base_scan', LaserScan, R.ranger_callback)
		# Observacion: Las funciones de callback se ejecutan cada vez que el programa recibe informacion 
		# desde un topico. Se utilizan, generalmente, para actualizar datos.
		# Pueden utilizarse funciones de callback programadas por el alumno, 
		# diferentes a las provistas por la clase Robot

		# Se recomienda que este ciclo vaya antes de todas las instrucciones de control
		# Sirve para asegurar que el Robot recibio su primera lectura de sensor
		while not rospy.is_shutdown() and len(R.distances)==0:
			R.rate.sleep()

		# Ciclo de ejemplo.
		# Si se va a controlar el robot con una funcion de robot_utilities en vez de con el ciclo:
		# opcion 1: cambiar True por False en la condicion del while
		# opcion 2: comentar el bloque de codigo
		while not rospy.is_shutdown() and False:
			# Define velocidad lineal [metros/segundo] del robot durante la iteracion
			#R.cmd.linear.x = 0.2
			#R.nSteps(1500,1500)

			# Velocidad angular [radianes/segundo]; rango aceptable: [-pi/2, pi/2 ], aprox. [-1.57, 1.57]
			R.cmd.angular.z = 0.0 

			# indica a Stage la velocidad que el robot tendra durante un ciclo
			R.cmd_pub.publish(R.cmd)
			
			# Ejemplo de como rescatar datos captados por los sensores
			# En este caso solo se imprimen, pero la idea es utilizarlos en decisiones de control
			print "_______________________"
			#print 'Primera lectura del laser [m]: '+ str(R.distances[0])
			#print 'Posicion robot [m]: x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y)
			#print 'Orientacion robot [grados]: ' + str(R.angle)
			R.show_distance()
			#print str(R.angle)
			# Mantiene la regularidad temporal (de acuerdo al rate definido en robot_utilities.Robot)
			R.rate.sleep()
			
		#print str(R.dintances)
		# Funciones de ejemplo (uncomment para usar): 
		#R.nSteps(1500,1500)
		#R.nSteps(1500,900)
		#R.nSteps(900,1500)
		#R.moveTill(math.pi*5.0/2, 0.5, 0.1)

		##CODIGO PARA ASSIGMENT 6
		n = numpy.asarray(R.distances)
		l = numpy.asarray(R.laser_angles)
		
		x=numpy.zeros(len(n))
		y=numpy.zeros(len(n))
		c = 0
		
		for i in range(len(n)):
			if n[i-1]<120:	
				x[c]=n[i-1]*math.cos(l[i-1]+(math.pi/2))
				y[c]=n[i-1]*math.sin(l[i-1]+(math.pi/2))
				c = 1 + c
		
		#plt.plot(x,y,'.', 0 , 0,'g^' )
		#plt.show()
		#print c   #cantidad de puntos


		##CODIGO PARA ASSIGMENT 8

		#DATOS
		K=15
		d=1.0
		Pg=0.1
		Pf=0.4
		#L=int(round((math.log(Pf))/(math.log(1-Pg**2))))
		L=100
		#print L

		#METODO
		
		for i in range (L):
			
			#1. 2 puntos random
			p1=0
			p2=0
			while p1==p2:				
				p1=random.randint(0,c-1)
				P2=random.randint(0,c-1)
			x1=x[p1]
			x2=x[p2]
			y1=y[p1]
			y2=y[p2]

			#2. Crear linea de ambos puntos
			A =y2-y1
			B =x1-x2
			C =y1*x2-y2*x1
			r =math.sqrt(A**2+B**2)

			#3. Encontrar puntos que hacen Match con la linea
			q=0
			for j in range(c):
				dd=numpy.fabs(A*x[j]+B*y[j]+C)/r
				if dd<d:
					q = q+1

			#4. Revisar si se logra el limite
			if q>=K:
				plt.plot((x1,x2),(y1,y2),color="red", linewidth=5.0, linestyle="-")
				plt.plot(x,y,'.', 0 , 0,'g^' )
				plt.show()
				print q
				print i+1
				break
			#6. Si falla
			print q
		
if __name__ == "__main__": Controller()
