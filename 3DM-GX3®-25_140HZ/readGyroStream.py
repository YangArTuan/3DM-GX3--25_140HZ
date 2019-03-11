import serial
import time
import binascii

ser = serial.Serial("/dev/ttyUSB0", 115200)
disable = [0x75, 0x65, 0x0c, 0x05, 0x05, 0x11, 0x01, 0x01, 0x00, 0x03, 0x19]
enable = [0x75, 0x65, 0x0c, 0x05, 0x05, 0x11, 0x01, 0x01, 0x01, 0x04, 0x1a]
ser.write(disable)
time.sleep(0.05)
ser.write(enable)


data = open("Staticdata.txt","a")
data.write("756580")
isHead = 0
dataBuff = ''
dataLen = 6
start = time.time()
while(1):
	tempBuff = ser.read_all()
	tempArray = binascii.b2a_hex(tempBuff)

	if isHead == 3 and len(tempArray) > 0: 
		data.write(tempArray)
		if(time.time()-start>7):
			data.close()
			break

	#print tempArray
	if isHead == 0 and tempArray == "75": isHead = 1
	if isHead == 1 and tempArray == "65": isHead = 2
	if isHead == 2 and tempArray == "80": isHead = 3


