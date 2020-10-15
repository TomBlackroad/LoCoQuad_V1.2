#!/usr/bin/python
import RPi.GPIO as GPIO
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
        super().__init__(filename)

        if(len(sys.argv)==2):
            print("EXECUTING TEST OF MOVEMENT", str(sys.argv[1])) 
            while True:
                super().executeMove(str(sys.argv[1]), 1)
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
        
        signal.signal(signal.SIGINT, self.close)
        self.state = mbl_bots.REST
        time.sleep(1)




#=============================================================================
# REST STATE
#=============================================================================
    def REST(self):
        print("CURRENT STATE: REST")
        start_time = time.time()
        while ((time.time()-start_time)<45):
            if(super().imu.detectCatch()):
                for i in range(3):
                    super().shake()
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
            3 : self.exploreReconTurn,
        }
        func = substates_explorelist.get(state, lambda:None)
        return func()

    def EXPLORE(self):
        print("CURRENT STATE: EXPLORE")
        super().flat()
        time.sleep(1)
        super().stand()
        #EXPLORING FiniteStateMachine
        while(self.state == mbl_bots.EXPLORE):
            self.exploreFSM(self.exploreState)
        

    def exploreGetData(self):
        print("CURRENT STATE: EXPLORE")
        print("CURRENT SUBSTATE: DATA ACQUISITION")
        self.distance = utils.getDistance()
        self.lastIMU = self.currentIMU
        self.currentIMU = super().imu.getImuRawData()
        start_time = time.time()
        self.frame = super().camera.getFrame()
        end_time = time.time()
        print('It took me {} us to get the frame' format(start_time-end_time))
        self.exploreState = mbl_bots.PROCESSDATA

    def exploreProcessData(self):
        print("CURRENT STATE: EXPLORE")
        print("CURRENT SUBSTATE: DATA PROCESSING")
        time.sleep(1)
        start_time = time.time()
        data = super().vision.analyze(self.frame)
        end_time = time.time()
        print('It took me {} us to process the frame' format(start_time-end_time))
        print("I am in coordinates: X={} Y={} T={}", format(data[0][1],data[0][2],data[0][3]))
        
        self.exploreState = mbl_bots.MOVE

    def exploreMove(self):
        print("CURRENT STATE: EXPLORE")
        print("CURRENT SUBSTATE: MOVING")
        #super().move(self.movesCode)
        #super().move(self.movesCode)
        super().camera.startVideo()
        time.sleep(3)
        start_time = time.time()
        while ((time.time()-start_time)<30):
            super().turnRight()
        #    super().walkFront()
        #super().camera.endVideo()
        time.sleep(1)    
        #pose_count = 0
        # while ((time.time()-start_time)<60):
        #     if(super().isBalanced()):
        #         super().balancePos(pose_count)
        #     else:
        #         super().balancePos(pose_count)
        #     if (pose_count >= 11):
        #         pose_count = 0 
        #     else:    
        #         pose_count = pose_count + 1
        #     time.sleep(0.5)
        # super().stand()
        # time.sleep(3)
        # super().walkFront()
        # super().walkFront()
        # super().walkFront()
        # super().walkFront()
        self.exploreState = mbl_bots.GETDATA
        self.state = mbl_bots.EXPLORE

    def exploreReconTurn(self):
        print("Not implemented...")
        





#=============================================================================
# SHOW OFF STATE
#=============================================================================
    def SHOWOFF(self):
        print("CURRENT STATE: SHOWOFF")
        #SHOWOFF METHODS
        super().swing()
        super().sayHello()
        self.state = mbl_bots.EXPLORE

#=============================================================================
# PHOTO STATE
#=============================================================================    
    def PHOTO(self):
        print("CURRENT STATE: PHOTO")
        super().cameraPose()
        super().camera.takePic()
        super().stand()
        self.state = mbl_bots.EXPLORE



    def close(self, signal, frame):
        super().camera.close()
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
