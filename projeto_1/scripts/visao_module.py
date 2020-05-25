#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import mobilenet_simples as mnet



def processa(frame):
    '''Use esta funcao para basear o processamento do seu robo'''

    result_frame, result_tuples = mnet.detect(frame)

    centro = (frame.shape[1]//2, frame.shape[0]//2)


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (int(point[0]) - int(length/2), int(point[1])),  (int(point[0]) + int(length/2), int(point[1])), color ,width, length)
        cv2.line(img_rgb, (int(point[0]), int(point[1]) - int(length/2)), (int(point[0]), int(point[1]) + int(length/2)),color ,width, length)

    cross(result_frame, centro, [255,0,0], 1, 17)

    # cv2.imshow('video', result_frame)
    # cv2.waitKey(1)

    return centro, result_frame, result_tuples



def identifica_cor(frame, cor):
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''

    # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao
    # vermelho puro (H=0) estão entre H=-8 e H=8.
    # Precisamos dividir o inRange em duas partes para fazer a detecção
    # do vermelho:
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    filtro1_verde = np.array([70, 255, 255], dtype=np.uint8)
    filtro2_verde = np.array([50, 50, 50], dtype=np.uint8)

    filtro1_rosa = np.array([160, 255, 255], dtype=np.uint8)
    filtro2_rosa = np.array([150, 50, 50], dtype=np.uint8)

    filtro1_azul = np.array([110, 255, 255], dtype=np.uint8)
    filtro2_azul = np.array([97, 100, 50], dtype=np.uint8)


    #Filtro para o creeper verde do PROJETO 1
    if cor == 'verde':
           
        segmentado_cor = cv2.inRange(frame_hsv, filtro2_verde, filtro1_verde)

    #Filtro para o creeper rosa do PROJETO 1
    elif cor == 'rosa':
        
        segmentado_cor = cv2.inRange(frame_hsv, filtro2_rosa, filtro1_rosa)

    #Filtro para o creeper azul do PROJETO 1
    elif cor == 'azul':
       
        segmentado_cor = cv2.inRange(frame_hsv, filtro2_azul, filtro1_azul)


    # Note que a notacão do numpy encara as imagens como matriz, portanto o enderecamento é
    # linha, coluna ou (y,x)
    # Por isso na hora de montar a tupla com o centro precisamos inverter, porque
    centro = (frame.shape[1]//2, frame.shape[0]//2)


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (int(point[0]) - int(length/2), int(point[1])),  (int(point[0]) + int(length/2), int(point[1])), color ,width, length)
        cv2.line(img_rgb, (int(point[0]), int(point[1]) - int(length/2)), (int(point[0]), int(point[1]) + int(length/2)),color ,width, length)



    # A operação MORPH_CLOSE fecha todos os buracos na máscara menores
    # que um quadrado 7x7. É muito útil para juntar vários
    # pequenos contornos muito próximos em um só.
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    # Encontramos os contornos na máscara e selecionamos o de maior área
    #contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    # Encontramos o centro do contorno fazendo a média de todos seus pontos.
    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        cross(frame, centro, [255,0,0], 1, 17)
    else:
        media = (0, 0)


    # cv2.imshow('video', frame)
    # cv2.imshow('seg', segmentado_cor)
    #cv2.waitKey(1)

    return media, maior_contorno_area
