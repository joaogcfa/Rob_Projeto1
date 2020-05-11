#! /usr/bin/env python
# -*- coding:utf-8 -*-

# rosrun projeto_1 main.py

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
import math
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage, LaserScan

import visao_module
import ponto_fuga
import segue_amarelo
import procura_amarelo
import garra_demo
import mobilenet_simples


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9



area = 0.0

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

def recebe(msg):
    
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id


	for marker in msg.markers:
		id = marker.id
		marcador = "ar_marker_" + str(id)

		#print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
		header = Header(frame_id=marcador)
		# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
		# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
		# Nao ser que queira levar angulos em conta
		trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		
		# Separa as translacoes das rotacoes
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# ATENCAO: tudo o que vem a seguir eh so para calcular um angulo
		# Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
		# no eixo X do robo (que e'  a direcao para a frente)
		t = transformations.translation_matrix([x, y, z])
		# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
		r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = numpy.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = math.degrees(math.acos(cosa))

		# Terminamos
		# print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))

distancia = 0
def scaneou(dado):
	global distancia
	distancia = dado.ranges[0]

cx = 0
bx = 0
maior_contorno_area=0
point_fuga = segue_amarelo.Follower()
point_fuga2 = procura_amarelo.Follower2()
#tutorial = garra_demo.MoveGroupPythonIntefaceTutorial()
#fecha_creeper = False
goal = ['azul', 0, 'bicycle']

lista_detect = None
img_detect = None

def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro

    global resultados
    global cx
    global bx
    global maior_contorno_area
    global cor
    global fecha_creeper
    global lista_detect
    global img_detect

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime
    delay = lag.nsecs


    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")       
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        
        cx = point_fuga.image_callback(temp_image)
        bx = point_fuga2.image_callback(temp_image)
        centro, img, resultados =  visao_module.processa(cv_image)
        media, maior_contorno_area = visao_module.identifica_cor(cv_image, goal[0])

        img_detect, lista_detect = mobilenet_simples.detect(cv_image)

        print(lista_detect)
        # fecha_creeper = tutorial.close_gripper()

        depois = time.clock()
        
        cv2.imshow("Camera", temp_image)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print('ex', e)


if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor2 = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos

    

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas

    crepeer_centralizado=False
    parado = False
    achou_amarelo_dnv = False
    ta_com_creeper = False 
    tolerancia = 15

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]
        # goal = ["blue", 23, "bird"]

    try:
        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
        
        while not rospy.is_shutdown():
            if len(media) != 0 and len(centro) != 0:


                # Centralizar na linha amarela
                if len(centro) != 0 and ta_com_creeper == False:
                    if cx > (centro[0] + tolerancia):
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                    if cx < (centro[0] - tolerancia):
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                    if (cx < (centro[0] + tolerancia) and cx > (centro[0]-tolerancia)):
                        vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)

                #Encontra creeper da cor escolhida
                while media[0] != 0 and parado == False:
                    if crepeer_centralizado == False:
            
                        if media[0] > (centro[0] + tolerancia):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                            velocidade_saida.publish(vel)
                        if media[0] < (centro[0] - tolerancia):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                            velocidade_saida.publish(vel)
                        if media[0] < (centro[0] + tolerancia) and media[0] > (centro[0]-tolerancia):
                            crepeer_centralizado = True
                    
                            
                    else:		

                        if len(media) != 0 and len(centro) != 0:

                            vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                            velocidade_saida.publish(vel)                            
                            
                            if distancia < 0.35:

                                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                                velocidade_saida.publish(velocidade)
                                rospy.sleep(0.6)
                                print("A DISTANCIA eh", distancia)
                                while distancia > 0.22:
                                    velocidade = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0))
                                    velocidade_saida.publish(velocidade)
                                    rospy.sleep(0.2)

                                    print("A DISTANCIA EH", distancia)

                                if distancia < 0.22:
                                    print("HI")
                                    velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                                    velocidade_saida.publish(velocidade)
                                    rospy.sleep(2.0)
                                    raw_input()
                                    ta_com_creeper = True
                                    parado = True
                        if not (media[0] < (centro[0] + tolerancia) and media[0] > (centro[0]-tolerancia)):
                                crepeer_centralizado = False
                                    
                
                while parado == True and bx == None:
                    print("Começa a girar")
                    velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0,math.pi/8.0))
                    velocidade_saida.publish(velocidade)
                    rospy.sleep(0.2)


                while bx != None and cx ==None:
                    if bx > (centro[0] + tolerancia):
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                    if bx < (centro[0] - tolerancia):
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                    if (bx < (centro[0] + tolerancia) and bx > (centro[0]-tolerancia)):
                        vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                
                # MobileNet aqui dentro
                while len(centro) != 0 and ta_com_creeper == True:

                    # [(objeto, bla, (bla,bla), (bla,bla)), (kldkfd)]

                
                    while len(lista_detect) == 0 or lista_detect[0][0] != goal[2]:
                        if cx > (centro[0] + tolerancia):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                        if cx < (centro[0] - tolerancia):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                        if (cx < (centro[0] + tolerancia) and cx > (centro[0]-tolerancia)):
                            vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
                        velocidade_saida.publish(vel)

                    while lista_detect[0][0] == goal[2] and len(lista_detect) != 0:
                        x_detect = (lista_detect[0][2][0] + lista_detect[0][3][0])/2
                        print("DISTÂNCIA DO BYCICLE: ", distancia)
                        if x_detect > (centro[0] + tolerancia):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                        if x_detect < (centro[0] - tolerancia):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                        if (x_detect < (centro[0] + tolerancia) and x_detect > (centro[0]-tolerancia)) and distancia > 0.4:
                            vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
                        if distancia <= 0.4:
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.5)
                            print("COÉ MENOR DEU CERTO DA UM ENTER NESSA BAGAÇA")
                            raw_input()


                        velocidade_saida.publish(vel)
                    


                        
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")