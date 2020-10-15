'''
	Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
	http://www.electronicwings.com
'''
import smbus                    #import SMBus module of I2C
from time import sleep          #import
import mbl_bots

class IMU:

	def __init__(self, bus):
		#bus = smbus.SMBus(3)    # or bus = smbus.SMBus(0) for older version boards
		self.bus = bus
		self.Device_Address = 0x68   # MPU6050 device address
		self.MPU_Init()
		print ("--> IMU Ready")


	def MPU_Init(self):
		#write to sample rate register
		self.bus.write_byte_data(self.Device_Address, mbl_bots.SMPLRT_DIV, 7)

		#Write to power management register
		self.bus.write_byte_data(self.Device_Address, mbl_bots.PWR_MGMT_1, 1)

		#Write to Configuration register
		self.bus.write_byte_data(self.Device_Address, mbl_bots.CONFIG, 0)

		#Write to Gyro configuration register
		self.bus.write_byte_data(self.Device_Address, mbl_bots.GYRO_CONFIG, 24)

		#Write to interrupt enable register
		self.bus.write_byte_data(self.Device_Address, mbl_bots.INT_ENABLE, 1)


	def read_raw_data(self, addr):
		#Accelero and Gyro value are 16-bit
		high = self.bus.read_byte_data(self.Device_Address, addr)
		low = self.bus.read_byte_data(self.Device_Address, addr+1)

		#concatenate higher and lower value
		value = ((high << 8) | low)

		#to get signed value from mpu6050
		if(value > 32768):
			value = value - 65536
		return value

	def getImuRawData(self):
		try:
	        #Read Accelerometer raw value
			acc_x = self.read_raw_data(mbl_bots.ACCEL_XOUT_H)
			acc_y = self.read_raw_data(mbl_bots.ACCEL_YOUT_H)
			acc_z = self.read_raw_data(mbl_bots.ACCEL_ZOUT_H)

	        #Read Gyroscope raw value
			gyro_x = self.read_raw_data(mbl_bots.GYRO_XOUT_H)
			gyro_y = self.read_raw_data(mbl_bots.GYRO_YOUT_H)
			gyro_z = self.read_raw_data(mbl_bots.GYRO_ZOUT_H)

	        #Full scale range +/- 250 degree/C as per sensitivity scale factor
			Ax = acc_x/16384.0
			Ay = acc_y/16384.0
			Az = acc_z/16384.0

			Gx = gyro_x/131.0
			Gy = gyro_y/131.0
			Gz = gyro_z/131.0

			data = [Ax,Ay,Az,Gx,Gy,Gz]

		except:
			data = [0,0,0,0,0,0]
			print("Error getting IMU data... something went wrong!!")
		
		return data

	def getStringImuRawData(self):
		data = self.getImuRawData()
		return "Gx=%.2f" %data[0], u'\u00b0'+ "/s", "\tGy=%.2f" %data[1], u'\u00b0'+ "/s", "\tGz=%.2f" %data[2], u'\u00b0'+ "/s", "\tAx=%.2f g" %data[3], "\tAy=%.2f g" %data[4], "\tAz=%.2f g" %data[5])
	
	def detectCatch(self, imu):
        data = self.getImuRawData()
        if(data[3] < 3 or data[4] < 3 or data[5] < 3): 
            return True
        else: 
            return False