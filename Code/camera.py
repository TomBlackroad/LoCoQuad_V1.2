from time import sleep
from picamera import PiCamera
from picamera.array import PiRGBArray
import datetime


class Cam(object):
	def __init__(self):
		self.camera = PiCamera()
		self.camera.resolution = (640, 480)
		self.camera.rotation = 180
		self.camera.framerate = 32
		self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
	
	def getFrame(self):
		self.camera.start_preview()
		self.camera.capture(self.rawCapture, use_video_port=True, format="bgr")
		self.frame = self.rawCapture.array
		print('Captured %dx%d image' % (self.frame.shape[1], self.frame.shape[0]))
		self.rawCapture.truncate(0)
		self.camera.stop_preview()
		return self.frame

	def getFrame2(self):
		for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
			# grab the raw NumPy array representing the image, then initialize the timestamp
			# and occupied/unoccupied text
			self.image = frame.array
			print('Captured %dx%d image' % (self.image.shape[1], self.image.shape[0]))
			# show the frame
			#cv2.imshow("Frame", image)
			#key = cv2.waitKey(1) & 0xFF
			# clear the stream in preparation for the next frame
			self.rawCapture.truncate(0)
			
			break
		return self.image

	def getFrame3(self):
		self.camera.capture(self.rawCapture, use_video_port=True, format="bgr")
		self.frame = self.rawCapture.array
		print('Captured %dx%d image' % (self.frame.shape[1], self.frame.shape[0]))
		self.rawCapture.truncate(0)
		return self.frame

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
		