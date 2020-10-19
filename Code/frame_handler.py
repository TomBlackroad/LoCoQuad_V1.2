#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
	File: frame_handler.py
	
	This class of the LoCo_benchmark project handles the frames and generates 
	useful outputs for the brain class to work with. 
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
import datetime 

from model import Model 
from frame_analyzer import Analyzer
import mbl_bots as ac

class Handler:

	def __init__(self):
		self.model = Model()
		self.frame_analyzer = Analyzer()

		self.frame_data = []
		self.frame_correct = None
		self.frame_id = None
		self.accepted_frame_number = 0
		self.deprecated_frame_number = 0
		self.without_tag_frame_number = 0
		self.tag_number_history = []
		self.original_frame = None # Initial FRAME value
		self.frame = None # Initial FRAME value
		self.frame_number = 0 # Initial
		self.ret = True

		self.real_dist_submatrix = None

		self.x_y_z_t_history = []

################################################################################

	def processFrame(self,frame):
		self.frame_id = "frame_" + datetime.datetime.now().strftime('%M:%S::%f')
		self.frame_data = []
		self.rvec_small = None
		self.tvec_small = None
		self.rvec_big = None
		self.tvec_big = None
		self.frame_number += 1
		self.original_frame = frame
		print(' ')
		print('Processing frame --> ' + self.frame_id)
		print('Frame Number = {}'.format(self.frame_number))
		print(' ')
		
		measured_T_C = []

		self.frame = cv2.cvtColor(self.original_frame, cv2.COLOR_BGR2GRAY)
		# print(self.original_frame)
		# print(' ')
		# print(self.frame)
		# print(' ')
		print('The Gray Scale frame from camera has size: {}x{}'.format(self.frame.shape[1],self.frame.shape[0]))
		print(' ')

		#-- Find all the aruco markers in the image
		self.corners_small, self.corners_big, self.ids_small, self.ids_big = self.frame_analyzer.findAllArUcoTags(self.frame)
		#-- Compute all poses from ids and corners
		ret_small, ret_big = self.frame_analyzer.poseAllArUcoTags(self.corners_small, self.corners_big, self.ids_small, self.ids_big)
		
		try:
			if ret_small is not None:
				print('SMALL TAGS DETECTED IN FRAME {}'.format(self.frame_number))
				print("# tags detected = " + str(self.frame_analyzer.small_tags_detected))
				self.rvec_small, self.tvec_small = ret_small[0], ret_small[1]
				for i in range(self.frame_analyzer.small_tags_detected):
					print("SMALL ID LIST:")
					msx,msy,msz = self.model.getT2C_X_Y_Z(self.rvec_small[i,0,:], self.tvec_small[i,0,:])
					#print("Measured XYZ: " + str(msx) + " " + str(msy) + " " + str(msz))
					index_sized = self.ids_small[i][0]
					print(index_sized)
					index_general = self.model.getIndex(self.ids_small[i],ac.SMALL_TAG_CODE)
					#print("Sized Index = " + str(index_sized) + " General Index = " + str(index_general))
					measured_T_C.append([ac.SMALL_TAG_CODE,index_sized,msx,msy,msz])
					#print("adding element OK")
					x,y,z,t = self.model.getX_Y_Z_T(ac.SMALL_TAG_CODE, self.ids_small[i], self.rvec_small[i,0,:], self.tvec_small[i,0,:])
					#print("X = " + str(x) + " Y = " + str(y) + " Z = " + str(z) + " Tita = " + str(t))
					self.frame_data.append([self.frame_number,x,y,z,t,ac.SMALL_TAG_CODE,self.ids_small[i],int(self.model.getIndex(self.ids_small[i],ac.SMALL_TAG_CODE))])
					if ac.WRITE_LOG_FILE:
						#print('\t (%d)[%4.0f]-- x=%4.2f y=%4.2f tita=%4.2f \r' %(ac.SMALL_TAG_CODE,self.ids_small[i][0],x,y,math.degrees(math.atan2(t[1],t[0]))))
						self.log.write('\t (%d){%d}[%d]-- x=%4.2f y=%4.2f tita=%4.2f \r' %(ac.SMALL_TAG_CODE,int(self.model.getIndex(self.ids_small[i],ac.SMALL_TAG_CODE)),int(self.ids_small[i][0]),x,y,math.degrees(t)))
			
			if ret_big is not None:
				print('BIG TAGS DETECTED IN FRAME {}'.format(self.frame_number))
				print("# tags detected = " + str(self.frame_analyzer.big_tags_detected))
				self.rvec_big, self.tvec_big = ret_big[0], ret_big[1]
				for i in range(self.frame_analyzer.big_tags_detected):
					print("BIG ID LIST:")
					mbx,mby,mbz = self.model.getT2C_X_Y_Z(self.rvec_big[i,0,:], self.tvec_big[i,0,:])
					#print("Measured XYZ: " + str(mbx) + " " + str(mby) + " " + str(mbz))
					index_sized = self.ids_big[i][0]
					print(index_sized)
					index_general = self.model.getIndex(self.ids_big[i],ac.BIG_TAG_CODE)
					#print("Sized Index = " + str(index_sized) + " General Index = " + str(index_general))
					measured_T_C.append([ac.BIG_TAG_CODE,index_sized,mbx,mby,mbz])
					#print("adding element OK")
					x,y,z,t = self.model.getX_Y_Z_T(ac.BIG_TAG_CODE, self.ids_big[i], self.rvec_big[i,0,:], self.tvec_big[i,0,:])
					#print("X = " + str(x) + " Y = " + str(y) + " Z = " + str(z) + " Tita = " + str(t))
					self.frame_data.append([self.frame_number,x,y,z,t,ac.BIG_TAG_CODE, self.ids_big[i],int(self.model.getIndex(self.ids_big[i],ac.BIG_TAG_CODE))])
					#self.x_y_t_history.append[self.frame_number,x,y,t,ac.BIG_TAG_CODE, ids_big[i]]
					if ac.WRITE_LOG_FILE:
						#print('\t (%d)[%4.0f]-- x=%4.2f y=%4.2f tita=%4.2f \r' %(ac.BIG_TAG_CODE,self.ids_big[i][0],x,y,math.degrees(math.atan2(t[1],t[0]))))
						self.log.write('\t (%d)[%d]-- x=%4.2f y=%4.2f tita=%4.2f \r' %(ac.BIG_TAG_CODE,int(self.ids_big[i]),x,y,math.degrees(t)))

			if ret_small == None and ret_big == None:
				self.without_tag_frame_number += 1
				self.tag_number_history.append([0])
				print('NO TAGS DETECTED IN FRAME {}'.format(self.frame_number))
				# if ac.WRITE_LOG_FILE:
				# 		#print('\t (%d)[%4.0f]-- x=%4.2f y=%4.2f tita=%4.2f \r' %(ac.BIG_TAG_CODE,self.ids_big[i][0],x,y,math.degrees(math.atan2(t[1],t[0]))))
				# 		self.log.write('\nNO TAGS DETECTED\n\n')

				self.frame_data.append([self.frame_number,0.0,0.0,0.0,0.0,2.0,999,999])

			elif ret_small == None:
				self.tag_number_history.append([len(ret_big)])
			elif ret_big == None:
				self.tag_number_history.append([len(ret_small)])
			else:
				self.tag_number_history.append([len(ret_big)+len(ret_small)])
		except:
			print('error and first part of frame processing')

		print(measured_T_C)

		confi_vector = None
		confi_vector = self.process_T_C_data(measured_T_C)
		if confi_vector == None:
			print('error creating CONFI_VECTOR')

		return self.frame_data, confi_vector

###############################################################################

	def process_T_C_data(self, measured_T_C):
		if len(measured_T_C)<1:
			print('No measured TC')
			self.real_dist_submatrix = None
			return None
		else:
			print('0.- # measured TC: ' + str(len(measured_T_C)))
			big_list = []
			small_list = []

			for i in range(len(measured_T_C)):
				if measured_T_C[i][0] == 0:
					index0 = measured_T_C[i][1]
					index1 = measured_T_C[i][2]
					index2 = measured_T_C[i][3]
					index3 = measured_T_C[i][4]
					big_list.append([index0,index1,index2,index3])
				elif measured_T_C[i][0] == 1:
					#print(self.model.getIndex(small_list[i][1],ac.SMALL_TAG_CODE))
					index0 = measured_T_C[i][1]
					index1 = measured_T_C[i][2]
					index2 = measured_T_C[i][3]
					index3 = measured_T_C[i][4]
					small_list.append([index0,index1,index2,index3])
			big_list = np.asarray(big_list, dtype=np.float32)
			small_list = np.asarray(small_list, dtype=np.float32)
			
			if len(big_list) > 0:
				print('1.- big info list len: ' + str(len(big_list)))
				big_list = big_list[big_list[:,0].argsort()]
			#big_list = np.argsort(big_list,axis=0)
			if len(small_list) > 0:
				print('2.- small info list len: ' + str(len(small_list)))
				small_list = small_list[small_list[:,0].argsort()]

			if big_list.size != 0 and small_list.size != 0:
				all_list = np.concatenate((big_list,small_list), axis=1)

			elif big_list.size != 0:
				all_list = big_list

			elif small_list.size != 0:
				all_list = small_list
			
			print('3.- done creating composed list')

			r_matrix_ids = []			

			for j in range(len(big_list)):
				index00 = self.model.getIndex(big_list[j][0],ac.BIG_TAG_CODE)
				r_matrix_ids.append(index00)

			for j in range(len(small_list)):
				index11 = self.model.getIndex(small_list[j][0],ac.SMALL_TAG_CODE)
				r_matrix_ids.append(index11)

			print('4.- done creating r_matrix_ids')

			r_matrix_ids = np.asarray(r_matrix_ids, dtype=np.int)
			m_matrix = self.createMeasuredDistMatrix(all_list)
			r_matrix = self.model.dist_matrix[:,r_matrix_ids]
			r_matrix = r_matrix[r_matrix_ids,:]

			dist_matrix = np.abs(m_matrix-r_matrix)/r_matrix
			confi_matrix = 1/np.cosh(5*dist_matrix)
			confi_vector =	{}

			confi_matrix = np.asarray(confi_matrix,dtype=np.float32)

			print('5.- done creating confi_matrix')

			for h in range(len(r_matrix_ids)):
				index000 = r_matrix_ids[h]
				index111 =  np.mean(np.concatenate((confi_matrix[h][:h],confi_matrix[h][(h+1):]), axis=0))
				confi_vector[str(index000)] = index111

			print('6.- done creating confi_vector')

			return confi_vector

###############################################################################

	def createMeasuredDistMatrix(self, index_list):
		matrix_dim = len(index_list)
		dist_matrix = np.zeros((matrix_dim, matrix_dim), dtype=np.float32)
		for i in range(matrix_dim):
			for j in range(matrix_dim):
				if i != j and dist_matrix[j,i] == 0.0:
					dist_matrix[i,j] = np.linalg.norm(index_list[i][1] - index_list[j][1])
		
		dist_matrix = dist_matrix + dist_matrix.T - np.diag(dist_matrix.diagonal())

		return dist_matrix

###############################################################################

	def processNextFrame(self,frame):
		print(' ')
		print('Processing Next Frame')
		print(' ')
		#print('SAVING FRAME IN CAPTURES FOLDER')
		#print(' ')
		#filename = ac.OUTPUT_PATH + str(self.frame_number)+datetime.datetime.now().strftime('__%H-%M-%S.jpg')
		#cv2.imshow(str(self.frame_number+1), frame)
		try:
			dataList, confi_vector = self.processFrame(frame)
			print('Frame ' + str(self.frame_number) + ' arrived correctly to the frame_handler!!')

		except:
			print('No information acquired from frame ' + str(self.frame_number))
			dataList = None
			confi_vector = None

		return dataList, confi_vector

###############################################################################





