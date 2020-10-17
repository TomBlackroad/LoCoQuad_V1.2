#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
	File: frame_full_analysis.py
	
	This piece of code from the LoCo_benchmark project analyses a input 
	video and outputs visual and numerical outputs. 
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

import sys, time, math, os, warnings

import scipy.signal as sci
import scipy.ndimage as sci2
import datetime 

import mbl_bots as ac
from frame_handler import Handler
#from ploter import Ploter


class Vision:

	def __init__(self):
		warnings.filterwarnings('ignore')

		self.frame_handler =  Handler()
		# self.ploter = Ploter(self.frame_handler.log)

		self.real_dist_matrix = self.frame_handler.model.createDistMatrix()
		# if ac.WRITE_LOG_FILE:
		# 	self.frame_handler.log.write('REAL DIST MATRIX:\n')
		# 	for i in range(self.real_dist_matrix.shape[0]):
		# 		for j in range(self.real_dist_matrix.shape[1]):
		# 			value =  "%4.2f  " %(self.real_dist_matrix[i][j])
		# 			self.frame_handler.log.write(value)
		# 		self.frame_handler.log.write(' \n')
	
		# self.analyze()

###############################################################################

	def applyMeanFilter(self, data):
		out = []

		if len(data) == 1:
			out = [[data[0][0],data[0][1],data[0][2],data[0][3],data[0][4],0.6]]
		else:
			frame_number = data[0][0]
			x_values = []
			y_values = []
			z_values = []
			t_values = []
			c_values = []

			for i in range(len(data)):
				x_values.append(data[i][1])
				y_values.append(data[i][2])
				z_values.append(data[i][3])
				t_values.append(data[i][4])
				c_values.append(data[i][5] + 0.2)


			out=[[frame_number,np.mean(x_values[i]),np.mean(y_values[i]),np.mean(z_values[i]),np.mean(t_values[i]),0.5]]
	
		return out

###############################################################################

	def applyDownsampleFilter(self, data):
		out = []
		out2 = []

		if len(data) == 1:
			out = [[data[0][0],data[0][1],data[0][2],data[0][3],data[0][4],0.6]]
		elif len(data) == 2:
			out = self.applyMeanFilter(data)
		else:
			downsample_rate = 2*(len(data)//3)
			frame_number = data[0][0]
			x_values = []
			y_values = []
			z_values = []
			t_values = []
			c_values = []

			for i in range(len(data)):
				x_values.append(data[i][1])
				y_values.append(data[i][2])
				z_values.append(data[i][3])
				t_values.append(data[i][4])
				c_values.append(data[i][5] + 0.2)

			for i in range(downsample_rate):
				index = self.detectWorstMessurement(x_values,y_values)
				del x_values[index]
				del y_values[index]
				del z_values[index]
				del t_values[index]
				del c_values[index]

			for i in range(len(data)-downsample_rate):
				out.append([frame_number,x_values[i],y_values[i],z_values[i],t_values[i],c_values[i]])

		out2 = self.applyMeanFilter(out)

		return out2

###############################################################################

	def applyConfiFilter(self, data, confi_vector):
		out = []
		if len(data) == 1:
			out = [[data[0][0],data[0][1],data[0][2],data[0][3],data[0][4],0.6]]
		elif len(data) == 2:
			out = self.applyMeanFilter(data)
		else:
			frame_number = data[0][0]
			x_values = []
			y_values = []
			z_values = []
			t_values = []
			id_values = []


			for i in range(len(data)):
				x_values.append(data[i][1])
				y_values.append(data[i][2])
				z_values.append(data[i][3])
				t_values.append(data[i][4])
				id_values.append(data[i][7])

			sumValue = 0
			x_confi = 0
			y_confi = 0
			z_confi = 0
			t_confi = 0
			confi = 0
			
			for j in range(len(id_values)):
				confi = float(confi_vector[str(id_values[j])])
				#print("confiabilidad de id" + str(id_values[i]) + " = " + str(confi))
				x_confi = x_confi + x_values[j] * confi
				y_confi = y_confi + y_values[j] * confi
				z_confi = z_confi + z_values[j] * confi
				t_confi = t_confi + t_values[j] * confi
				sumValue = sumValue + confi

			x_confi = x_confi/sumValue
			y_confi = y_confi/sumValue
			z_confi = z_confi/sumValue
			t_confi = t_confi/sumValue

			out = [[frame_number,x_confi,y_confi,z_confi,t_confi,0.6]]

		return out

###############################################################################

	def processData(self, data, confi):
		processed_data = applyConfiFilter(data, confi)*3
		processed_data += applyDownsampleFilter(data, confi)		
		processed_data /= 4

		return processed_data		

###############################################################################

	def analyze(self,frame):

		data = []
		processed_data = []

		try:
			data, confi_vector = self.frame_handler.processNextFrame(frame)
			print('    //////////////////////////////////////////     ')
			print('Data from frame -->')
			print(data)
			print(' ')
			print(confi_vector)
			processed_data = self.processData(data, confi_vector)

		except:
			processed_data = None
			err_message = 'ERROR in frame ' + str(self.frame_handler.frame_number) + '\n'
			print(err_message)

		return processed_data

###############################################################################
###############################################################################
