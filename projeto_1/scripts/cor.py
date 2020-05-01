#! /usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cormodule

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

maior_area = 0

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	# print("frame")
	global cv_image
	global media
	global centro
	global maior_area
	
	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	# print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		# print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		# cv_image = cv2.flip(cv_image, -1)
		# área: maior_area
		media, centro, maior_area =  cormodule.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)


# Inserir o código que vai receber o evento do laser. E também guardar a distância 
# do dado.ranges[0] - tem no le_scan.py

distancia = 0
def scaneou(dado):
	global distancia
	#distancia = np.array(dado.ranges).round(decimals=2)[0]
	distancia = dado.ranges[0]
	# print("A DISTANCIA eh no def", distancia)
if __name__=="__main__":
	rospy.init_node("cor")

	# topico_imagem = "/kamera"
	topico_imagem = "/camera/rgb/image_raw/compressed"
	
	# Para renomear a *webcam*
	#   Primeiro instale o suporte https://github.com/Insper/robot19/blob/master/guides/debugar_sem_robo_opencv_melodic.md
	#
	#	Depois faça:
	#	
	#	rosrun cv_camera cv_camera_node
	#
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	#
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
	# 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	## trazer / criar um Subscriber do tópico /scan - le_scan.py
	
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	#print("Usando ", topico_imagem)
	
	ta_no_centro = False
	chegou_distancia = False

	menor_media = 305
	maior_media = 345

	try:
		while not chegou_distancia:
			while not rospy.is_shutdown():
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				if ta_no_centro == False: 
					if len(media) != 0 and len(centro) != 0:
						
						# print("Média dos verdes: {0}, {1}".format(media[0], media[1]))
						# print("Centro dos verdes: {0}, {1}".format(centro[0], centro[1]))
						# print("A DISTANCIA É", distancia)


						# Identificar onde está o verde e manter centralizado
						if (media[0] > centro[0]):
							vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
						if (media[0] < centro[0]):
							vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
						if ((menor_media<media[0]) and (media[0]<maior_media)):
							ta_no_centro = True

					velocidade_saida.publish(vel)
					rospy.sleep(0.1)

				else:		

					if len(media) != 0 and len(centro) != 0:

						vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))

						# print("Média: {0}, {1}".format(media[0], media[1]))
						# print("Centro: {0}, {1}".format(centro[0], centro[1]))
						# print("A DISTANCIA EH", distancia)

						
						if distancia < 0.50:

							velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
							velocidade_saida.publish(velocidade)
							rospy.sleep(0.6)
							print("A DISTANCIA eh", distancia)
							while distancia > 0.23:
								velocidade = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
								velocidade_saida.publish(velocidade)
								rospy.sleep(0.2)

								print("A DISTANCIA EH", distancia)

							while distancia < 0.23:
								print("HI")
								velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
								velocidade_saida.publish(velocidade)
								vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
								rospy.sleep(0.2)
								break								

						if not ((menor_media<media[0]) and (media[0]<maior_media)):
							ta_no_centro = False

					velocidade_saida.publish(vel)
					rospy.sleep(0.25)

	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")
