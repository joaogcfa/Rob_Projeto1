#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function

# import the necessary packages
import numpy as np
import argparse
import cv2

import rospkg
import os
import math

def ponto_de_fuga(ang1, ang2, linear1, linear2):
    if ang2 == ang1:
        ang2 = ang1 + 1 # para nao dar div por zero
    x_fuga = (-(linear2 - linear1))/(ang2 - ang1)
    y_fuga = ang1*x_fuga + linear1
    return (x_fuga, y_fuga)  

def calcula_coef_ang(A1, A2):
    coef_ang = (A2[1]-A1[1])/(A2[0]-A1[0])
    return coef_ang
                      
def calcula_coef_linear (A1, A2):
    coef_ang = (A2[1]-A1[1])/(A2[0]-A1[0])
    coef_linear = A1[1]-(coef_ang*A1[0])
    return coef_linear

def coloca_ponto(frame):
    # Our operations on the frame come here
    rgb = frame #  cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Mascara branca
    cap1 = np.array([249])
    cap2 = np.array([255])
    mask = cv2.inRange(gray, cap1, cap2)
    #cv2.imshow('mask', mask)
    

    mask = cv2.blur(mask, (3,3))
    
    hough_img_rgb2 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    min_contrast = 100
    max_contrast = 200
    linhas = cv2.Canny(hough_img_rgb2, min_contrast, max_contrast)
    
    lines = cv2.HoughLinesP(linhas, 10, math.pi/180.0, 100, np.array([]), 45, 5)

    if lines is not None:
        a,b,c = lines.shape

        hough_img_rgb = cv2.cvtColor(linhas, cv2.COLOR_GRAY2BGR)

        coeficientes_angular=[]
        coeficientes_linear=[]
            
        for i in range(a):
            # Faz uma linha ligando o ponto inicial ao ponto final, com a cor vermelha (BGR)
            cv2.line(hough_img_rgb, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 5, cv2.LINE_AA)
            coef_angular = calcula_coef_ang((lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]))
            coeficientes_angular.append(coef_angular)
                        
            coef_linear = calcula_coef_linear((lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]))
            coeficientes_linear.append(coef_linear)
                            
        for c in range(len(coeficientes_angular)):
            if coeficientes_angular[c] == min(coeficientes_angular) and coeficientes_angular[c]!=0:
                menor = coeficientes_angular[c]
                i_menor = c
        maior = max(coeficientes_angular) 
        # i_menor = coeficientes_angular.index(c)
        i_maior = coeficientes_angular.index(maior)
                        
        linear_menor = coeficientes_linear[i_menor]                  
        linear_maior = coeficientes_linear[i_maior]
        
        
        PFx, PFy = ponto_de_fuga(maior, menor, linear_maior, linear_menor)
        
        # DESENHANDO PONTO DE FUGA
        cor=(255, 255, 0)
        cv2.circle(frame, (int(PFx), int(PFy)), 20, cor)
        cv2.circle(frame, (int(PFx),int(PFy)), 4, cor, 1)
        
        #cv2.line(frame, (lines[i][0][0], lines[i][0][1]), (int(PG[0]),int(PG[1])), cor,3)
        return PFx, PFy

    return [(0,0), (0,0)]

    
