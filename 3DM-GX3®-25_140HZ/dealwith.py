from BojayIMUAPI import BojayIMUAPI
import time
import struct
import binascii
import serial
from serial import *
import threading
from threading import Lock

myBojayIMUAPI = BojayIMUAPI()

port_plc = myBojayIMUAPI.OpenPort_IMUAPIClass('/dev/ttyUSB1', 0)
port_sensor = myBojayIMUAPI.OpenPort_IMUAPIClass('/dev/ttyUSB0', 1)
time.sleep(0.7)



myBojayIMUAPI.SensorSetToIdel_IMUAPIClass()

myBojayIMUAPI.SensorSetEulerRPY_IMUAPIClass()
#time.sleep(0.2)

ret = myBojayIMUAPI.DynamicGetAccelerationAndAngle_IMUAPIClass(myBojayIMUAPI.x_axis,0, 180,True,113)

#time.sleep(4)
while(myBojayIMUAPI.DynamicIsReadyGetDatalist_IMUAPIClass() ==False):
	time.sleep(0.01)
	continue
# time.sleep(0.2)

# myLocalThread_2 = threading.Thread(target=thread_2)
# myLocalThread_2.start()

ret = myBojayIMUAPI.StaticGetAccelerationAndAngle_IMUAPIClass(2)


# time.sleep(2.5)
while(myBojayIMUAPI.StaticIsReadyGetDatalist_IMUAPIClass()==False):
	time.sleep(0.01)
	continue
#print len(myBojayIMUAPI.DynamicangularVelocityList)
f1 = open("22_1.csv","w")
f1.write('AnglesRateX' + ',' + 'AnglesRateY' + ',' + 'AnglesRateZ' + '\n')
for i in range(0,len(myBojayIMUAPI.DynamicangularVelocityList),1):
	#print "111"
	#print myBojayIMUAPI.DynamicangularVelocityList[i][1]
	f1.writelines(str(myBojayIMUAPI.DynamicangularVelocityList[i][0]) +','+ str(myBojayIMUAPI.DynamicangularVelocityList[i][1])+','+str(myBojayIMUAPI.DynamicangularVelocityList[i][2])+"\n")
f1.close()

f2 = open("22_2.csv","w")
f2.write('AccelerationX' + ',' + 'AccelerationY' + ',' + 'AccelerationZ' + '\n')
for i in range(0,len(myBojayIMUAPI.DynamicaccelerationList),1):
	#print "111"
	#print myBojayIMUAPI.DynamicangularVelocityList[i][1]
	f2.writelines(str(myBojayIMUAPI.DynamicaccelerationList[i][0]) +','+ str(myBojayIMUAPI.DynamicaccelerationList[i][1])+','+str(myBojayIMUAPI.DynamicaccelerationList[i][2])+"\n")
f2.close()

f3 = open("22_3.csv","w")
f3.write('AccelerationX' + ',' + 'AccelerationY' + ',' + 'AccelerationZ' + '\n')
for i in range(0,len(myBojayIMUAPI.StaticaccelerationList),1):
	#print "111"
	#print myBojayIMUAPI.DynamicangularVelocityList[i][1]
	f3.writelines(str(myBojayIMUAPI.StaticaccelerationList[i][0]) +','+ str(myBojayIMUAPI.StaticaccelerationList[i][1])+','+str(myBojayIMUAPI.StaticaccelerationList[i][2])+"\n")
f3.close()

f4 = open("22_4.csv","w")
f4.write('AnglesRateX' + ',' + 'AnglesRateY' + ',' + 'AnglesRateZ' + '\n')
for i in range(0,len(myBojayIMUAPI.StaticangularVelocityList),1):
	#print "111"
	#print myBojayIMUAPI.DynamicangularVelocityList[i][1]
	f4.writelines(str(myBojayIMUAPI.StaticangularVelocityList[i][0]) +','+ str(myBojayIMUAPI.StaticangularVelocityList[i][1])+','+str(myBojayIMUAPI.StaticangularVelocityList[i][2])+"\n")
f4.close()
time.sleep(1)