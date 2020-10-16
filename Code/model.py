#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
	File: model.py
	
	This class of the LoCo_benchmark project represents the RING through 
	Homogeneous Transformation Matrices 
"""

__author__ = "Manuel Bernal"
__credits__ = ["Manuel Bernal", "Ignacio Cuiral", "Javier Antoran",
                    "Javier Civera"]
__license__ = "GPLv3"
__version__ = "1.0.1"
__maintainer__ = "Manuel Bernal"
__email__ = "mbl@unizar.es"
__status__ = "Prototype"


import cv2
import cv2.aruco as aruco
import numpy as np

import sys, time, math, os

import scipy.signal as sci
import scipy.ndimage as sci2 

import analysis_configurator as ac


class Model:

	def __init__(self, ring_size=4, n_of_faces=4, n_of_tag_sices=2):

		self.size = ac.RING_SIZE 
		self.number_of_faces = ac.NUMBER_FACES 
		self.number_of_tag_sices = ac.NUMBER_TAG_SICES

		self.space = ac.RING_SIZE*ac.NUMBER_FACES*ac.NUM_TAGS_SIZE1
		self.space1 = ac.RING_SIZE*ac.NUM_TAGS_SIZE1
		self.space2 = ac.RING_SIZE*ac.NUM_TAGS_SIZE2

		self.dist_matrix = None

		self.T_ABS_F = None
		self.T_F_P = None
		self.T_P_T = None
		self.T_C_R = None
		self.createModel()

##############################################################################

	def createHomTransformationMatrix(self, x, y, z = 0, axis = None, angle = None):
		Tmatrix = np.identity(4, dtype=np.float32)
		rotation = np.identity(3, dtype=np.float32)
		
		#ROTATION

		if axis == 'x':
			rotation[0,0]= 1.0
			rotation[1,1]= math.cos(math.radians(angle))
			rotation[1,2]= -math.sin(math.radians(angle))
			rotation[2,1]= math.sin(math.radians(angle))
			rotation[2,2]= math.cos(math.radians(angle))

		elif axis == 'y':
			rotation[0,0]= math.cos(math.radians(angle))
			rotation[0,2]= math.sin(math.radians(angle))
			rotation[1,1]= 1.0
			rotation[2,0]= -math.sin(math.radians(angle))
			rotation[2,2]= math.cos(math.radians(angle))

		elif axis == 'z':
			rotation[0,0]= math.cos(math.radians(angle))
			rotation[0,1]= -math.sin(math.radians(angle))
			rotation[1,0]= math.sin(math.radians(angle))
			rotation[1,1]= math.cos(math.radians(angle))
			rotation[2,2]= 1.0

		np.copyto(Tmatrix[:3,:3], rotation)

		#TRANSLATION
		translation = np.array([[x,y,z]])
		np.copyto(Tmatrix[:3,3:], translation.T)

		#SCALE
		Tmatrix[3,3]= 1.0

		return Tmatrix

##############################################################################

	def createT_Abs_F(self):
		T_ABS_F = []
		T_ABS_F.append(self.createHomTransformationMatrix(0,0,0,'y',90))
		T_ABS_F.append(self.createHomTransformationMatrix(0,self.size*ac.A4_WIDTH,0,'x',90))
		T_ABS_F.append(self.createHomTransformationMatrix(self.size*ac.A4_WIDTH,self.size*ac.A4_WIDTH,0,'y',-90))
		T_ABS_F.append(self.createHomTransformationMatrix(self.size*ac.A4_WIDTH,0,0,'x',-90))

		return T_ABS_F 

##############################################################################

	def createT_F_P(self):
		T_F_P = [] 
		for i in range(self.number_of_faces):
			T_F_P.append([])
		for j in range(self.size):
			T_F_P[0].append(self.createHomTransformationMatrix(0,j*ac.A4_WIDTH,0,'z',90))
			T_F_P[1].append(self.createHomTransformationMatrix(j*ac.A4_WIDTH,0,0))
			T_F_P[2].append(self.createHomTransformationMatrix(0,-j*ac.A4_WIDTH,0,'z',-90))
			T_F_P[3].append(self.createHomTransformationMatrix(-j*ac.A4_WIDTH,0,0,'z',180))

		return T_F_P 

##############################################################################

	def createT_P_T(self):
		T_P_T = [] 
		for i in range(self.number_of_tag_sices):
			T_P_T.append([])

		T_P_T[0].append(self.createHomTransformationMatrix(7.5,15,0))	# Tag1 ... (big up left)
		T_P_T[0].append(self.createHomTransformationMatrix(22.5,15,0))	# Tag2 ... (big up right)

		T_P_T[1].append(self.createHomTransformationMatrix(5,6,0))		# Tag3 ... (small down left)
		T_P_T[1].append(self.createHomTransformationMatrix(15,6,0))		# Tag4 ... (small down mid)
		T_P_T[1].append(self.createHomTransformationMatrix(25,6,0))		# Tag5 ... (small down right)
		
		return T_P_T 

##############################################################################

	def createT_C_R(self):
		T_C_R = self.createHomTransformationMatrix(0,4.5,-3.8,'x',90)
		return T_C_R 

##############################################################################

	def createModel(self):
		self.T_ABS_F = self.createT_Abs_F()
		self.T_F_P = self.createT_F_P()
		self.T_P_T = self.createT_P_T()
		self.T_C_R = self.createT_C_R()

##############################################################################

	def createDistMatrix(self):
		matrix_dim = ac.RING_SIZE * ac.NUMBER_FACES * (ac.NUM_TAGS_SIZE1+ac.NUM_TAGS_SIZE2) 
		dist_matrix = np.zeros((matrix_dim, matrix_dim), dtype=np.float32)
		for i in range(matrix_dim):
			for j in range(matrix_dim):
				if i != j and dist_matrix[j,i] == 0.0:
					dist_matrix[i,j] = np.linalg.norm(self.getVector(i) - self.getVector(j))
		
		dist_matrix = dist_matrix + dist_matrix.T - np.diag(dist_matrix.diagonal())

		self.dist_matrix = dist_matrix
		

		return dist_matrix

##############################################################################

	def getVector(self, index):

		if index < self.space:
			id_i = (index//self.space1)*10 + index%self.space1 
			id1 = int(id_i//10)
			id2 = int((id_i%10)//2)
			id3 = int(id_i%2)
			id4 = 0
		else:
			i2 = index-self.space
			id_i = (i2//self.space2)*20 + i2%self.space2
			id1 = int(id_i//20)
			id2 = int((id_i%20)//3)
			id3 = int(id_i%3)
			id4 = 1

		matrix = self.T_ABS_F[id1].dot(self.T_F_P[id1][id2]).dot(self.T_P_T[id4][id3])
		vec = matrix[:3,3:]
		return vec

##############################################################################

	def getIndex(self, ids, size_code):
		index = None
		index_0 = None
		if size_code == ac.BIG_TAG_CODE:
			index = (ids//10)*self.space1 + ids%10 

		elif size_code == ac.SMALL_TAG_CODE:
			index_0 = (ids//20)*self.space2 + ids%20
			index = index_0+self.space
		return index

##############################################################################

	def magnitude(self, vec1, vec2):
		return np.linalg.norm(vec1 - vec2)

##############################################################################

	def getT_T_C(self, rot, tras):
		R_tc = np.matrix(cv2.Rodrigues(rot)[0]).T
		pos = -R_tc*np.matrix(tras).T

		T_T_C = np.zeros((4,4), dtype=np.float32)
		#ROTATION
		np.copyto(T_T_C[:3,:3], R_tc)

		#TRANSLATION
		pos_aux = np.array([[pos[0],pos[1],pos[2]]])
		np.copyto(T_T_C[:3,3:], pos_aux.T)

		#SCALE
		T_T_C[3,3]= 1.0

		return T_T_C

##############################################################################

	def getT_ABS_ROBOT(self, size, id_number, R, t):
		T_T_C = self.getT_T_C(R,t)
		T_ABS_ROBOT = None

		#big tags
		if size == ac.BIG_TAG_CODE and id_number < 40:
			id1 = int(id_number//10)
			id2 = int((id_number%10)//2)
			id3 = int(id_number%2)
			id4 = 0

		#small tags
		if size == ac.SMALL_TAG_CODE:
			id1 = int(id_number//20)
			id2 = int((id_number%20)//3)
			id3 = int(id_number%3)
			id4 = 1
		
		T_ABS_ROBOT = self.T_ABS_F[id1].dot(self.T_F_P[id1][id2]).dot(self.T_P_T[id4][id3]).dot(T_T_C).dot(self.T_C_R)   

		return T_ABS_ROBOT

##############################################################################

	def getX_Y_Z_T(self, size, id_number, R, t):
		T_ABS_ROBOT = self.getT_ABS_ROBOT(size, id_number, R, t)
		if T_ABS_ROBOT is not None:
			tita = math.atan2(T_ABS_ROBOT[1, 0],T_ABS_ROBOT[0, 0])
			x = T_ABS_ROBOT[0,3] 
			y = T_ABS_ROBOT[1,3]
			z = T_ABS_ROBOT[2,3]

			return (x,y,z,tita)

		else:
			return (0.0,0.0,0.0,0.0)


##############################################################################

	def getT2C_X_Y_Z(self, R, t):
		T_T_C = self.getT_T_C(R,t)

		if T_T_C is not None:
			x = T_T_C[0,3] 
			y = T_T_C[1,3]
			z = T_T_C[2,3]

			return (x,y,z)

		else:
			return (0.0,0.0,0.0)