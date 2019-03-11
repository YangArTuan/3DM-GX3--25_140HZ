#from BojayIMUAPI import BojayIMUAPI
import time
import struct
import binascii
import serial
from serial import *
import threading




#second step
readtxt = open('DynamicData.txt', 'r')
read_value = readtxt.read()
replace_value = read_value
readtxt.close()
replace_value = replace_value.replace('\n','')
t = replace_value.split('7565802')
a = list(t)
del a[0]
del a[len(a) - 1]
a.pop()
value = []
for i in a:
    value.append("7565802"+i)
for j in range(0,len(a),1):
	print value[j]
print len(value)
dt = open("Staticdt1.txt","w")
for i in range(0,len(value),1):
	dt.write(str(value[i]) + '\n')
dt.close()
time.sleep(2)
print "finish second step!"


#third step
tmp = []
accelerationDataList = []
anglesRateDataList = []
ref_readings = []
mean_ref_readings = []

opendt1 = open("Staticdt1.txt","r")
count = len(opendt1.readlines())
# print count
opendt1.close()
time.sleep(0.2)
opendt = open("Staticdt1.txt","r")
for i in range(0,count,1):
    t = opendt.readline()
    if len(t)>92:
        tmp.append(t)
print len(tmp)
for j in range(0,len(tmp),1):
    sourceData = tmp[j].decode()
    #print sourceData
    temp = struct.unpack('!f', sourceData[40:48].decode('hex'))[0]
    accelerationDataList.append(round(temp, 4))
    temp = struct.unpack('!f', sourceData[48:56].decode('hex'))[0]
    accelerationDataList.append(round(temp, 4))
    temp = struct.unpack('!f', sourceData[56:64].decode('hex'))[0]
    accelerationDataList.append(round(temp, 4))
    temp = struct.unpack('!f', sourceData[68:76].decode('hex'))[0]
    anglesRateDataList.append(round(temp, 4))
    temp = struct.unpack('!f', sourceData[76:84].decode('hex'))[0]
    anglesRateDataList.append(round(temp, 4))
    temp = struct.unpack('!f', sourceData[84:92].decode('hex'))[0]
    anglesRateDataList.append(round(temp, 4))

    ref_readings.append(accelerationDataList)
    mean_ref_readings.append(anglesRateDataList)

    accelerationDataList = []
    anglesRateDataList = []
Acceleration = open('static_Acceleration.csv',"w")
Acceleration.write('AccelerationX' + ',' + 'AccelerationY' + ',' + 'AccelerationZ' + '\n')
AnglesRate = open('static_AnglesRate.csv',"w")
AnglesRate.write('AnglesRateX' + ',' + 'AnglesRateY' + ',' + 'AnglesRateZ' + '\n')
for i in range(0,len(ref_readings),1):
	Acceleration.write(str(ref_readings[i][0]) + ',' + str(ref_readings[i][1]) + ',' + str(ref_readings[i][2]) + '\n')
Acceleration.close()
for j in range(0,len(mean_ref_readings),1):
	AnglesRate.write(str(mean_ref_readings[j][0]) + ',' + str(mean_ref_readings[j][1]) + ',' + str(mean_ref_readings[j][2]) + '\n')

AnglesRate.close
opendt.close()
print "make it!"
