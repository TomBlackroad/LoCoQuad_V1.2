#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
	File: frame_analyzer.py
	
	This class of the LoCo_benchmark project contains all the ArUco related functionalities.
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

import mbl_bots as ac


class Analyzer():

	def __init__(self):
		self.generateArUcoDictionaries()
		self.getCalibrationAndDistorsionFiles()
		#--- Define Tag Sizes
		self.TAG_SIZE_SMALL = ac.TAG1_SIZE 
		self.TAG_SIZE_BIG = ac.TAG2_SIZE 

###############################################################################

	def generateArUcoDictionaries(self):
		#--- Define the ArUco dictionaries & params
		self.aruco_dict_small  = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
		self.aruco_dict_big  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
		self.parameters  = aruco.DetectorParameters_create()

###############################################################################

	def getCalibrationAndDistorsionFiles(self):
		#--- Get the camera calibration path
		calib_path  = ac.CALIBRATION_PATH
		self.cam_matrix   = np.loadtxt(calib_path+'/cameraMatrix.txt', delimiter=',')
		self.cam_distortion   = np.loadtxt(calib_path+'/cameraDistortion.txt', delimiter=',')

###############################################################################

	def findAllArUcoTags(self, frame):
		corners_small, ids_small, _ = aruco.detectMarkers(image=frame, 
							dictionary=self.aruco_dict_small, parameters=self.parameters,
							cameraMatrix=self.cam_matrix, distCoeff=self.cam_distortion)
		corners_big, ids_big, _ = aruco.detectMarkers(image=frame, 
							dictionary=self.aruco_dict_big, parameters=self.parameters,
							cameraMatrix=self.cam_matrix, distCoeff=self.cam_distortion)

		if ac.TAG_NUMBER_LIMITED:
			if len(corners_small) > ac.MAX_TAG_NUMBER:
				corners_small = corners_small[:ac.MAX_TAG_NUMBER]
				ids_small = ids_small[:ac.MAX_TAG_NUMBER]
			if len(corners_big) > ac.MAX_TAG_NUMBER:
				corners_big = corners_big[:ac.MAX_TAG_NUMBER]
				ids_big = ids_big[:ac.MAX_TAG_NUMBER]

		return corners_small, corners_big, ids_small, ids_big

###############################################################################

	def poseAllArUcoTags(self, corners_small, corners_big, ids_small, ids_big):
		if ids_small is not None:
			self.small_tags_detected = len(ids_small)
			ret_small = aruco.estimatePoseSingleMarkers(corners_small, self.TAG_SIZE_SMALL, 
						self.cam_matrix, self.cam_distortion)
		else:
			self.small_tags_detected = 0
			ret_small = None

		if ids_big is not None:
			self.big_tags_detected = len(ids_big)
			ret_big = aruco.estimatePoseSingleMarkers(corners_big, self.TAG_SIZE_BIG, 
						self.cam_matrix, self.cam_distortion)
		else: 
			self.big_tags_detected = 0
			ret_big = None

		return ret_small, ret_big

###############################################################################



