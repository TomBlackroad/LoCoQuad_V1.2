#!/usr/bin/python
import RPi.GPIO as GPIO
import numpy as np
import time
import math
import smbus
import sys
import utils
import logging
import mbl_bots
import signal
import os

from random import randint

from robot import Robot



class LoCoQuad(Robot):
    def __init__(self):
        self.dir = os.path.dirname(__file__)
        filename = os.path.join(self.dir, mbl_bots.ROBOTFILE)
        print("Directory: " + self.dir)
        print("RobotFileDirectory: " + filename)        
        super(LoCoQuad, self).__init__(filename)

        if(len(sys.argv)==2):
            print("EXECUTING TEST OF MOVEMENT", str(sys.argv[1])) 
            while True:
                super(LoCoQuad, self).executeMove(str(sys.argv[1]), 1)
        else: 
            while True:
                self.generalFSM(self.state)


#=============================================================================
# GENERAL FSM
#=============================================================================
    def generalFSM(self, state):
        states_list = {
            0 : self.INIT,
            1 : self.REST,
            2 : self.EXPLORE,
            3 : self.SHOWOFF,
            4 : self.PHOTO,
        }
        func = states_list.get(state, lambda:None)
        return func()

#=============================================================================
# INIT STATE
#=============================================================================
    def INIT(self):
        print("CURRENT STATE: INIT")
        GPIO.setmode(GPIO.BCM) # use Raspberry Pi board pin numbers
        self.lastIMU = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.currentIMU = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.lastdata = -1
        signal.signal(signal.SIGINT, self.close)
        self.state = mbl_bots.REST
        time.sleep(1)




#=============================================================================
# REST STATE
#=============================================================================
    def REST(self):
        print("CURRENT STATE: REST")
        start_time = time.time()
        while ((time.time()-start_time)<mbl_bots.REST_TIME):
            if(self.imu.detectCatch()):
                for i in range(3):
                    super(LoCoQuad, self).shake()
                break
            else:
                time.sleep(0.2)
        time.sleep(1)
        self.state = mbl_bots.EXPLORE





#=============================================================================
# EXPLORE STATE & FSM
#=============================================================================
    def exploreFSM(self, state):
        substates_explorelist = {
            0 : self.exploreGetData,
            1 : self.exploreProcessData,
            2 : self.exploreMove,
        }
        func = substates_explorelist.get(state, lambda:None)
        return func()

    def EXPLORE(self):
        print("CURRENT STATE: EXPLORE")
        super(LoCoQuad, self).flat()
        self.exploreTime = time.time()
        #self.camera.startVideo()

        time.sleep(1)
        super(LoCoQuad, self).stand()
        #EXPLORING FiniteStateMachine
        while(self.state == mbl_bots.EXPLORE):
            self.exploreFSM(self.exploreState)
        #self.camera.endVideo()
        

    def exploreGetData(self):
        print("CURRENT STATE: EXPLORE")
        print("CURRENT SUBSTATE: DATA ACQUISITION")
        self.distance = utils.getDistance()
        self.lastIMU = self.currentIMU
        self.currentIMU = self.imu.getImuRawData()
        start_time = time.time()
        self.frame = self.camera.getFrame()
        self.camera.truncateFrame()
        end_time = time.time()
        print('It took me {} s to get the frame'.format(start_time-end_time))
        self.exploreState = mbl_bots.PROCESSDATA

    def exploreProcessData(self):
        print("CURRENT STATE: EXPLORE")
        print("CURRENT SUBSTATE: DATA PROCESSING")
        start_time = time.time()
        data = self.vision.analyze(self.frame)
        end_time = time.time()
        print(' ')
        print('It took me {} s to process the frame'.format(start_time-end_time))
        if data is not None:    
            print("I am in coordinates: X={} Y={} T={}".format(data[0][1],data[0][2],np.degrees(data[0][3]+mbl_bots.PI2)))
            self.lastdata = data
        else:
            print("Ohh!! I couldn't find my coordinates...")
            if self.lastdata != -1:
                print("I was in coordinates: X={} Y={} T={} last time (frame: {})".format(self.lastdata[0][1],self.lastdata[0][2],np.degrees(self.lastdata[0][3]+mbl_bots.PI2),self.lastdata[0][0]))

        self.exploreState = mbl_bots.MOVE

    def exploreMove(self):
        print("CURRENT STATE: EXPLORE")
        print("CURRENT SUBSTATE: MOVING")
        #super(LoCoQuad, self).move(self.movesCode)
        #super(LoCoQuad, self).move(self.movesCode)
        

        if ((time.time()-self.exploreTime)>mbl_bots.EXPLORATION_TIME):
            self.exploreState = mbl_bots.GETDATA
            self.state = mbl_bots.REST

        else:
            start_time = time.time()
            #super(LoCoQuad, self).turnRight()
            end_time = time.time()
            print(' ')
            print('It took me {} s to move'.format(start_time-end_time))
            self.exploreState = mbl_bots.GETDATA
            self.state = mbl_bots.EXPLORE
            
            
        





#=============================================================================
# SHOW OFF STATE
#=============================================================================
    def SHOWOFF(self):
        print("CURRENT STATE: SHOWOFF")
        #SHOWOFF METHODS
        super(LoCoQuad, self).swing()
        super(LoCoQuad, self).sayHello()
        self.state = mbl_bots.EXPLORE

#=============================================================================
# PHOTO STATE
#=============================================================================    
    def PHOTO(self):
        print("CURRENT STATE: PHOTO")
        super(LoCoQuad, self).cameraPose()
        self.camera.takePic()
        super(LoCoQuad, self).stand()
        self.state = mbl_bots.EXPLORE



    def close(self, signal, frame):
        #self.camera.close()
        print("\nTurning off LoCoQuad Activity...\n")
        GPIO.cleanup() 
        sys.exit(0)

    


#                                                             #     
#  ---------------------------------------------------------  #
#  ---------------------------------------------------------  #
#  ---------------------------------------------------------  #
#            _____                    _____                   #
#           /\    \                  /\    \                  #
#          /::\    \                /::\    \                 #   
#         /::::\    \               \:::\    \                #  
#        /::::::\    \               \:::\    \               #   
#       /:::/\:::\    \               \:::\    \              #   
#      /:::/__\:::\    \               \:::\    \             #
#     /::::\   \:::\    \              /::::\    \            #     
#    /::::::\   \:::\    \    ____    /::::::\    \           #     
#   /:::/\:::\   \:::\    \  /\   \  /:::/\:::\    \          #  
#  /:::/  \:::\   \:::\____\/::\   \/:::/  \:::\____\         #   
#  \::/    \:::\  /:::/    /\:::\  /:::/    \::/    /         #       
#   \/____/ \:::\/:::/    /  \:::\/:::/    / \/____/          #       
#            \::::::/    /    \::::::/    /                   #         
#             \::::/    /      \::::/____/                    #     
#             /:::/    /        \:::\    \                    #     
#            /:::/    /          \:::\    \                   #     
#           /:::/    /            \:::\    \                  #    
#          /:::/    /              \:::\____\                 #  
#          \::/    /                \::/    /                 #  
#           \/____/                  \/____/                  #    
#                                                             #  
#  ---------------------------------------------------------  #
#  ---------------------------------------------------------  #
#  ---------------------------------------------------------  #
#                                                             #   

if __name__=='__main__':
  LoCoQuad = LoCoQuad()
