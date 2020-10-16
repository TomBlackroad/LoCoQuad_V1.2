from time import sleep
from picamera import PiCamera
from picamera.array import PiRGBArray
import datetime


class Cam(object):
	def __init__(self):
		self.camera = PiCamera()
		self.camera.resolution = (640, 480)
		self.camera.rotation = 180
		self.camera.framerate = 10
		self.rawCapture = PiRGBArray(self.camera)
	
	def getFrame(self):
		self.camera.capture(self.rawCapture, format="bgr")
		frame = self.rawCapture.array
		print('Captured %dx%d image' % (frame.shape[1], frame.shape[0]))
		return frame

	def truncateFrame(self):
		self.rawCapture.truncate(0)

	def takePic(self, output_dir="./Captures/"):
		self.camera.start_preview()
		sleep(2)
		filename = output_dir + datetime.datetime.now().strftime('%Y-%m-%d--%H-%M-%S.jpg')
		print(filename)
		self.camera.capture(filename)
		self.camera.stop_preview()

	def startVideo(self, output_dir="./Captures/"):
		filename = output_dir + datetime.datetime.now().strftime('%Y-%m-%d--%H-%M-%S.h264')
		print(filename)
		self.camera.start_recording(filename)

	def endVideo(self):
		self.camera.stop_recording()
		