# **************************************************************#
#Author:Tony
#Mail : tony_dong@zhbojay.com
#Release Note:
#Version:V1.0   2018/6/14
#the initial version


#Version:V1.2   2018/7/3
#1.modify the offset function
#2.add the easy put DUT function
#3.add read sensor function


#Version:V1.3   2018/7/6
#1.add function to set PLC offset
#2.Change the returen value: 1 t0 -1,since some state is 1
#3:Check the door sensor


#Version:V1.4   2018/11/28
#adapt 3DM-GX3-15

#Version:V1.5   2019/1/5
#add acceleration & angular velocity

#Version:V1.6	2019/1/17
#add stream model
#Mail:719294349@qq.com
# **************************************************************#

import serial
import serial.tools.list_ports
import binascii
import struct
import time
import os
import threading

class BojayIMUAPI:

    # **************************************************************#
    #definition variable:
    strErrorMessage = "ok"
    myPLCSerialPort = None #PLC port
    mySensorSerialPort = None #Sensor port
    myPLCSerialPortName = "" #PLC port
    mySensorSerialPortName = "" #Sensor port

    PLCVersion = ""
    SwitchPLC = 0
    SwitchSensor = 1

    x_axisMaxLimit = 180
    x_axisMinLimit = -180
    y_axisMaxLimit = 180
    y_axisMinLimit = -180
    z_axisMaxLimit = 180
    z_axisMinLimit = -180


    x_axis = 100
    y_axis = 101
    z_axis = 102
    xy_axis = 103
    xyz_axis = 104
    max_limit = 105
    min_limit = 106

    PLCSensorStart = 107
    PLCSensorReset = 108
    PLCSensorStop = 109
    PLCSensorUrgentstop = 110
    PLCSensorCheckDoor = 111

    DUT1SenSor = 117
    DUT2SenSor = 118
    DUT3SenSor = 119
    DUT4SenSor = 120

    absoluteMove = 112
    relativeMove = 113

    XAxisBest_180 = 0
    XAxisBest_0 = 0
    XAxisBest_90_Plus = 0
    XAxisBest_90_Minus = 0

    YAxisBest_0 = 0
    YAxisBest_90_Plus = 0
    YAxisBest_90_Minus = 0

    #0: x:0   1:x:180  2:x:90  3:x:-90  4:y:90  5:y:-90
    SensorOffset = [0,0,0,0,0,0]

    #for sensor
    # when open the SW the command
    openSWStepOne = [0x75, 0x65, 0x7f, 0x04, 0x04, 0x10, 0x02, 0x01, 0x74, 0xbe]
    openSWStepTwo = [0x75, 0x65, 0x01, 0x02, 0x02, 0x07, 0xe6, 0xcc]
    # set to idle
    openSWStepThree = [0x75, 0x65, 0x01, 0x06, 0x02, 0x02, 0x02, 0x04, 0x02, 0x03, 0xf0, 0x82]

    stepOne = [0x75, 0x65, 0x0c, 0x10, 0x10, 0x35, 0x01, 0x00, 0x0a, 0x01, 0x03, 0x0f, 0x11, 0x00, 0x0a, 0x00, 0x0a,
               0x00,
               0x00, 0x00, 0x7e, 0xc8]
    # stepTwo = [0x75, 0x65, 0x0c, 0x0f, 0x0f, 0x37, 0x01, 0xb9, 0xe5, 0xa3, 0x5d, 0xb7, 0x27, 0xc5, 0xac, 0xba, 0x1e,
    #            0x98,
    #            0xdd, 0x76, 0xad]
    stepTwo = [0x75, 0x65, 0x0c, 0x0f, 0x0f, 0x37, 0x01, 0xb9, 0x8c, 0x82, 0x5a, 0xba, 0x12,
               0x89 , 0xdb, 0x3a, 0x46, 0xb4, 0x85, 0x46, 0xd5]
    # stepThree = [0x75, 0x65, 0x0c, 0x07, 0x07, 0x08, 0x01, 0x01, 0x0c, 0x00, 0x0a, 0x14, 0x35]
    #stepThree = [0x75, 0x65, 0x0c, 0x0d, 0x0d, 0x08, 0x01, 0x03, 0x0c, 0x00, 0x0a, 0x04, 0x00, 0x0a, 0x05, 0x00, 0x0a, 0x3f, 0xbc]
    stepThree = [0x75, 0x65, 0x0c, 0x0d, 0x0d, 0x08, 0x01, 0x03, 0x0c, 0x00, 0x01, 0x04, 0x00, 0x01, 0x05, 0x00, 0x01, 0x24, 0x50]
    stepFour = [0x75, 0x65, 0x01, 0x02, 0x02, 0x06, 0xe5, 0xcb]

    #start to acquire
    startAcquire = [0x75, 0x65, 0x0c, 0x09, 0x05, 0x11, 0x01, 0x01, 0x01, 0x04, 0x11, 0x02, 0x01, 0x20, 0x9a]

    #stop acquire
    stopAcquire = [0x75, 0x65, 0x0c, 0x09, 0x05, 0x11, 0x01, 0x01, 0x00, 0x04, 0x11, 0x02, 0x01, 0x1f, 0x95]


    sensorContiuneData = [0x75, 0x65, 0x80, 0x0e, 0x0e, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x8c, 0x21]

    # single get data
    singleGetData = [0x75, 0x65, 0x0c, 0x04, 0x04, 0x01, 0x01, 0x00, 0xf0, 0xdc]

    #clear default setting
    clearSettingThree = [0x75, 0x65, 0x0c, 0x04, 0x04, 0x08, 0x01, 0x00, 0xf7, 0xf1]
    clearSettingFour = [0x75, 0x65, 0x01, 0x02, 0x02, 0x06, 0xe5, 0xcb]


    #***************3DM-GX3-15 commands***************
    communicationMode = [0x75, 0x65, 0x7f, 0x04, 0x04, 0x10, 0x02, 0x01, 0x74, 0xbe]
    setToIdle = [0x75, 0x65, 0x01, 0x02, 0x02, 0x02, 0xe1, 0xc7]
    resume = [0x75, 0x65, 0x01, 0x02, 0x02, 0x06, 0xe5, 0xcb]

    pollData = [0x75, 0x65, 0x0c, 0x04, 0x04, 0x01, 0x01, 0x00, 0xf0, 0xdc]
    #format data of measurement, acceleration discriptor(0x04), angular rate discriptor(0x05)
    messageFormat = [0x75, 0x65, 0x0c, 0x0a, 0x0a, 0x08, 0x01, 0x02, 0x04, 0x00, 0x0a, 0x05, 0x00, 0x0a, 0x22, 0xa0]
    enableDataStream = [0x75, 0x65, 0x0c, 0x05, 0x05, 0x11, 0x01, 0x01, 0x01, 0x04, 0x1a]
    disableDataStream = [0x75, 0x65, 0x0c, 0x05, 0x05, 0x11, 0x01, 0x01, 0x00, 0x03, 0x19]
    loadDefaultSettings = [0x75, 0x65, 0x0c, 0x03, 0x03, 0x30, 0x05, 0x21, 0x47]

    #***************3DM-GX3-15 commands***************



    # **************************************************************#
    def __init__(self):
        try:
            #PLC rotate thread
            self.PlCThread = threading.Thread(target=self.PLCThreadFunction)
            self.bStartRotatePLC = False
            self.PlCThread.start()
            self.StartAngle = 0
            self.EndAngle = 0
            self.bStartFromCurrentAngle = False
            self.PLCRotateAxis = 0
            self.PLCMoveMode = 0

            self.AcquireDataMode = 0 #0 is dynamic 1 is static
            self.AcquireSensorDataThread = threading.Thread(target=self.AcquireSensorDataThreadFunction)
            self.bStartAcquireData = False
            self.AcquireSensorDataThread.start()
            self.DynamicangularVelocityList = []
            self.DynamicaccelerationList = []
            self.DynamicAcquireFinish = False

            self.DynamicDelayTime = 0
            self.StaticDelayTime = 0

            self.StaticreadingTime = 0
            self.StaticangularVelocityList = []
            self.StaticaccelerationList = []

        except:
            self.strErrorMessage = "__init__ except error"
            return -1

    # **************************************************************#

    # **************************************************************#
    def PLCThreadFunction(self):
        try:
            while True:
                if self.bStartRotatePLC == False:
                    time.sleep(0.01)
                    continue
                else:
                    #Dynamic mode
                    if self.AcquireDataMode == 0:
                        #start delay
                        DynamicdelayTimeBegin = time.time()
                        while True:
                            if time.time() - DynamicdelayTimeBegin >= self.DynamicDelayTime:
                                break
                            else:
                                time.sleep(0.001)
                                continue

                        self.mySensorSerialPort.write(self.enableDataStream)
                        print "start to dynamic mode"
                        if self.bStartFromCurrentAngle == False:
                            ret = self.SetMoveDegree_IMUAPIClass(self.PLCRotateAxis,self.StartAngle,10)
                            if ret != 0:
                                print "PLCRotateThreadFunction->SetMoveDegree_IMUAPIClass error"
                                return -1
                            #waiting the moving stop
                            myWaitTime = 0
                            while (myWaitTime < 10):
                                time.sleep(0.05)
                                ret = self.GetmoveSignal_IMUAPIClass(self.PLCRotateAxis)
                                if ret == -1:
                                    self.strErrorMessage = "SetMoveDegree_IMUAPIClass:wait finish single time out"
                                    return -1
                                elif ret == 1:
                                    myWaitTime = myWaitTime + 0.05
                                else:
                                    break

                        #PLC start to rotate, sensor start to acquire data
                        self.bStartAcquireData = True
                        time.sleep(0.001)


                        if self.PLCMoveMode == self.absoluteMove:
                            ret = self.SetMoveDegree_IMUAPIClass(self.PLCRotateAxis,self.EndAngle,10)
                            if ret != 0:
                                print "PLCRotateThreadFunction->SetMoveDegree_IMUAPIClass error"
                                return -1
                        elif self.PLCMoveMode == self.relativeMove:
                            ret = self.StepMove_IMUAPIClass(self.PLCRotateAxis,self.EndAngle)
                            if ret != 0:
                                print "PLCRotateThreadFunction->StepMove_IMUAPIClass error"
                                return -1


                        # waiting the moving stop
                        myWaitTime = 0
                        while (myWaitTime < 10):
                            time.sleep(0.1)
                            ret = self.GetmoveSignal_IMUAPIClass(self.PLCRotateAxis)
                            print "get ret=%d"%(ret)
                            if ret == -1:
                                self.strErrorMessage = "SetMoveDegree_IMUAPIClass:wait finish single time out"
                                return -1
                            elif ret == 1:
                                myWaitTime = myWaitTime + 0.1
                            else:
                                break

                        #PLC end to rotate, sensor end to acquire data
                        #wait action finish
                        time.sleep(0.15)
                        self.bStartAcquireData = False
                        self.DynamicAcquireFinish = True
                        time.sleep(0.01)
                        print "start unpack!"
                        unpack = 0
                        while len(self.DynamicangularVelocityList) == 0:
                            ret = self.FastDealMyData(unpack)
                        print "dynamic len=%d" % (len(self.DynamicangularVelocityList))
                        print "end to dynamic mode"
                        self.mySensorSerialPort.write(self.disableDataStream)
                        

                    #Static mode 
                    elif self.AcquireDataMode == 1:
                        #start delay
                        StaticdelayTimeBegin = time.time()
                        while True:
                            if time.time() - StaticdelayTimeBegin >= self.StaticDelayTime:
                                break
                            else:
                                time.sleep(0.001)
                                continue

                        self.mySensorSerialPort.write(self.enableDataStream)
                        print "start to static mode"
                        #Start to acquire static data
                        self.StaticStartTime = time.time()
                        print "StaticreadingTime=%d,StaticreadingTime=%d"%(self.StaticStartTime,self.StaticreadingTime)
                        self.StaticangularVelocityList = []
                        self.StaticaccelerationList = []
                        self.bStartAcquireData = True
                        time.sleep(0.001)
                        while ((time.time() - self.StaticStartTime) <= self.StaticreadingTime):
                            time.sleep(0.001)
                            continue
                        #time.sleep(0.01)
                        self.bStartAcquireData = False
                        self.StaticAcquireFinish = True
                        time.sleep(0.01)
                        unpack = 1
                        while len(self.StaticaccelerationList) == 0:
                            ret = self.FastDealMyData(unpack)
                        print "static len =%d" % (len(self.StaticaccelerationList))
                        print "end to static mode"
                        self.mySensorSerialPort.write(self.disableDataStream)

                    self.bStartRotatePLC = False
        except:
            self.strErrorMessage = "PLCRotateThreadFunction except error"
            return -1
    # **************************************************************#



    # **************************************************************#
    def AcquireSensorDataThreadFunction(self):
        try:
            while True:
                if self.bStartAcquireData == False:
                    time.sleep(0.01)
                    continue
                else:
                    if self.AcquireDataMode == 1:  # 1 is static data
                        isHead = 0
                        StaticData = open("StaticData.txt", "w")
                        StaticData.write("756580")
                        while True:
                            tempBuff = self.mySensorSerialPort.read_all()
                            tempArray = binascii.b2a_hex(tempBuff)

                            if isHead == 3 and len(tempArray) > 0:
                                StaticData.write(tempArray)
                                if self.StaticAcquireFinish == True:
                                    StaticData.close()
                                    print "finish static"
                                    break
                            if isHead == 0 and tempArray == "75": isHead = 1
                            if isHead == 1 and tempArray == "65": isHead = 2
                            if isHead == 2 and tempArray == "80": isHead = 3

                    else:
                        isHead = 0
                        DynamicData = open("DynamicData.txt", "w")
                        DynamicData.write("756580")
                        while True:
                            tempBuff = self.mySensorSerialPort.read_all()
                            tempArray = binascii.b2a_hex(tempBuff)
                            if isHead == 3 and len(tempArray) > 0:
                                DynamicData.write(tempArray)
                                if self.DynamicAcquireFinish == True:
                                    DynamicData.close()
                                    print "finish dynamic"
                                    break
                            if isHead == 0 and tempArray == "75": isHead = 1
                            if isHead == 1 and tempArray == "65": isHead = 2
                            if isHead == 2 and tempArray == "80": isHead = 3
        except:
            self.strErrorMessage = "AcquireSensorDataFunction except error"
            return -1
    # **************************************************************#

    def FastDealMyData(self, model):
        try:
            if model == 1:
                # one step
                readtxt = open('StaticData.txt', 'r')
                read_value = readtxt.read()
                replace_value = read_value
                readtxt.close()
                replace_value = replace_value.replace('\n', '')
                t = replace_value.split('7565802')
                a = list(t)
                del a[0]
                del a[len(a) - 1]
                a.pop()
                value = []
                for i in a:
                    value.append("7565802" + i)
                print len(value)
                dt = open("Staticdt1.txt", "w")
                for i in range(0, len(value), 1):
                    dt.write(str(value[i]) + '\n')
                dt.close()

                # third step
                tmp = []
                accelerationDataList = []
                anglesRateDataList = []
                ref_readings = []
                mean_ref_readings = []

                opendt1 = open("Staticdt1.txt", "r")
                count = len(opendt1.readlines())
                opendt1.close()
                opendt = open("Staticdt1.txt", "r")
                for i in range(0, count, 1):
                    t = opendt.readline()
                    if len(t) > 92:
                        tmp.append(t)
                print len(tmp)
                for j in range(0, len(tmp), 1):
                    sourceData = tmp[j].decode()
                    # print sourceData
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

                self.StaticangularVelocityList = mean_ref_readings
                self.StaticaccelerationList = ref_readings

            elif model == 0 :
                # one step
                readtxt = open('DynamicData.txt', 'r')
                read_value = readtxt.read()
                replace_value = read_value
                readtxt.close()
                replace_value = replace_value.replace('\n', '')
                t = replace_value.split('7565802')
                a = list(t)
                del a[0]
                del a[len(a) - 1]
                a.pop()
                value = []
                for i in a:
                    value.append("7565802" + i)
                print len(value)
                dt = open("Dynamicdt1.txt", "w")
                for i in range(0, len(value), 1):
                    dt.write(str(value[i]) + '\n')
                dt.close()

                # third step
                tmp = []
                accelerationDataList = []
                anglesRateDataList = []
                ref_readings = []
                mean_ref_readings = []

                opendt1 = open("Dynamicdt1.txt", "r")
                count = len(opendt1.readlines())
                opendt1.close()
                opendt = open("Dynamicdt1.txt", "r")
                for i in range(0, count, 1):
                    t = opendt.readline()
                    if len(t) > 92:
                        tmp.append(t)
                print len(tmp)
                for j in range(0, len(tmp), 1):
                    sourceData = tmp[j].decode()
                    # print sourceData
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

                self.DynamicangularVelocityList = mean_ref_readings
                self.DynamicaccelerationList = ref_readings
            return 0
        except:
            self.strErrorMessage = "DealMyDataPack except error"
            return -1



    # **************************************************************#
    def DynamicGetAccelerationAndAngle_IMUAPIClass(self,axis,startAngle,EndAngle,bStartFromCurrentAngle=True,MoveMode=112,delay=4.5):
        try:
            self.StartAngle = startAngle
            self.EndAngle = EndAngle
            self.bStartFromCurrentAngle = bStartFromCurrentAngle
            self.PLCRotateAxis = axis
            self.PLCMoveMode = MoveMode

            #Clear data
            self.DynamicaccelerationList = []
            self.DynamicangularVelocityList = []
            self.DynamicAcquireFinish = False

            #Start to move PLC
            self.AcquireDataMode = 0 # 0 is dynamic mode
            self.bStartRotatePLC = True

            #dut delay
            self.DynamicDelayTime = delay
        except:
            self.strErrorMessage = "DynamicGetAccelerationAndAngle_IMUAPIClass except error"
            return -1
    # **************************************************************#

    # **************************************************************#
    def DynamicIsReadyGetDatalist_IMUAPIClass(self):
        try:
            if self.DynamicAcquireFinish == False:
                return False
            else:
                time.sleep(0.05)
        except:
            self.strErrorMessage = "DynamicIsReadyGetDatalist_IMUAPIClass except error"
            return -1
    # **************************************************************#


    # **************************************************************#
    def StaticGetAccelerationAndAngle_IMUAPIClass(self,StaticreadingTime,delay=4.5):
        try:
            self.StaticreadingTime = StaticreadingTime

            #Clear data
            self.StaticangularVelocityList = []
            self.StaticaccelerationList = []
            self.StaticAcquireFinish = False

            #Start to move PLC
            self.AcquireDataMode = 1  #1 is static mode
            self.bStartRotatePLC = True

            #dut delay
            self.StaticDelayTime = delay
        except:
            self.strErrorMessage = "StaticGetAccelerationAndAngle_IMUAPIClass except error"
            return -1
    # **************************************************************#

    # **************************************************************#
    def StaticIsReadyGetDatalist_IMUAPIClass(self):
        try:
            if self.StaticAcquireFinish == False:
                return False
            else:
                time.sleep(0.05)
        except:
            self.strErrorMessage = "StaticIsReadyGetDatalist_IMUAPIClass except error"
            return -1
    # **************************************************************#





    # **************************************************************#
    #open the port
    def OpenPort_IMUAPIClass(self,portName,switch):
        try:
            if(switch == self.SwitchPLC):
                myseiral = serial.Serial(port=portName,
                                        timeout=0.05,
                                        baudrate=115200,
                                        parity=serial.PARITY_ODD)

            elif(switch == self.SwitchSensor):
                myseiral = serial.Serial(port=portName,
                                        baudrate=115200,
                                        timeout=0.02)

            myseiral.close()
            myseiral.open()
            if myseiral.is_open == False:
                self.strErrorMessage = "OpenPort_IMUAPIClass:open " + portName + " fail"
                return -1

            if switch == self.SwitchPLC:
                self.myPLCSerialPort =  myseiral
                self.myPLCSerialPortName = portName
                ret = self.SignalReset_IMUAPIClass(20)
                if ret != 0:
                    return -1
            elif switch == self.SwitchSensor:
                self.mySensorSerialPort = myseiral
                self.mySensorSerialPortName = portName
                ret = self.LoadSensorOffset_IMUAPIClass()
                if ret != 0:
                    return -1
                    
            return 0
        except:
            self.strErrorMessage = "OpenPort_IMUAPIClass except error"
            return -1


    # **************************************************************#
    #close the port
    def ClosePort_IMUAPIClass(self,switch):
        try:
            if switch == self.SwitchPLC:
                if(self.myPLCSerialPort.is_open):
                    self.myPLCSerialPort.close()
                    self.myPLCSerialPort = None
            elif switch == self.SwitchSensor:
                if(self.mySensorSerialPort.is_open):
                    self.mySensorSerialPort.close()
                    self.mySensorSerialPort = None
            return 0
        except:
            self.strErrorMessage = "ClosePort_IMUAPIClass except error"
            return -1

    # **************************************************************#
    #Get PLC veresion
    def GetPLCVersion_IMUAPIClass(self):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "GetPLCVersion_IMUAPIClass:the port did not open"
                return -1
            command = "%01#RDD003000030256"
            PLcver = self.__readVer(command)
            if(len(PLcver) < 1):
                self.strErrorMessage = "GetPLCVersion_IMUAPIClass:get version fail"
                return -1
            PLCVersion = str(PLcver)
            return PLCVersion
        except:
            self.strErrorMessage = "GetPLCVersion_IMUAPIClass except error"
            return -1



    # **************************************************************#
    #Set speed
    #:axis : x/y/z
    #value:the speed
    def SetAxisSpeed_IMUAPIClass(self,ofWhatAxis,Value):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "SetAxisSpeed_IMUAPIClass:the port did not open"
                return -1

            command = ""
            if ofWhatAxis == self.x_axis:
                command = "%01#WDD0020000201"
                Value = Value / 5
            elif ofWhatAxis == self.y_axis:
                command = "%01#WDD0021000211"
                Value = Value / 5
            elif ofWhatAxis == self.z_axis:
                command = "%01#WDD0022000221"
                Value = Value / 2

            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SetAxisSpeed_IMUAPIClass:set speed fail"
                return -1
            return 0
        except:
            self.strErrorMessage = "SetAxisSpeed_IMUAPIClass except error"
            return -1


    # **************************************************************#
    #Get speed
    #:ofWhatAxis : x/y/z
    def GetAxisSpeed_IMUAPIClass(self,ofWhatAxis):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "GetAxisSpeed_IMUAPIClass:the port did not open"
                return -1

            command = ""
            if ofWhatAxis == self.x_axis:
                command = "%01#RDD0020000201"
            elif ofWhatAxis == self.y_axis:
                command = "%01#RDD0021000211"
            elif ofWhatAxis == self.z_axis:
                command = "%01#RDD0022000221"

            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            self.myPLCSerialPort.write(command)

            #read data
            readString = self.ReadData(0.1)
            if("fail" in readString):
                self.strErrorMessage = "GetAxisSpeed_IMUAPIClass:read data timeout"
                return -1
            value = self.__getValueOfByte(readString)

            if ofWhatAxis == self.x_axis:
                return int(value) * 5
            elif ofWhatAxis == self.y_axis:
                return int(value) * 5
            elif ofWhatAxis == self.z_axis:
                return int(value) * 2

        except:
            self.strErrorMessage = "GetAxisSpeed_IMUAPIClass except error"
            return -1


    # **************************************************************#
    #Get axis finish single
    #ofWhatAxis:
    def GetmoveSignal_IMUAPIClass(self, ofWhatAxis):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "GetmoveSignal_IMUAPIClass:the port did not open"
                return -1

            if ofWhatAxis == self.x_axis:
                command = "%01#RCSR0052"
            elif ofWhatAxis == self.y_axis:
                command = "%01#RCSR0054"
            elif ofWhatAxis == self.z_axis:
                command = "%01#RCSR0056"
            elif ofWhatAxis == self.xy_axis:
                command = "%01#RCSR005C"
            elif ofWhatAxis == self.xyz_axis:
                command = "%01#RCSR0064"

            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            self.myPLCSerialPort.write(command)
            readString =  self.ReadData(0.01)
            if("fail" in readString):
                self.strErrorMessage = "GetmoveSignal_IMUAPIClass:read data fail"
                return -1
            readString = int(readString[6])
            if (readString == 1):
                return 0
            elif (readString == 0):
                return 1
        except:
            self.strErrorMessage = "GetmoveSignal_IMUAPIClass except error"
            return -1



    # **************************************************************#
    #Set degree
    #:ofWhatAxis : x/y/z
    #Value: the degree what you want to move
    #timeout:the max wait time
    def SetMoveDegree_IMUAPIClass(self,ofWhatAxis,Value,timeout):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "SetMoveDegree_IMUAPIClass:the port did not open"
                return -1
            #clear
            if ofWhatAxis == self.x_axis:
                command = "%01#WCSR002C0"
            elif ofWhatAxis == self.y_axis:
                command = "%01#WCSR002D0"
            elif ofWhatAxis == self.z_axis:
                command = "%01#WCSR002E0"
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SetMoveDegree_IMUAPIClass:" +  command + "__writeRead fail"
                return -1


            command = ""
            if ofWhatAxis == self.x_axis:
                command = "%01#WDD0020200203"
            elif ofWhatAxis == self.y_axis:
                command = "%01#WDD0021200213"
            elif ofWhatAxis == self.z_axis:
                command = "%01#WDD0022200223"


            #limit
            if ofWhatAxis == self.x_axis:
                if Value > self.x_axisMaxLimit:
                    Value = self.x_axisMaxLimit
                if Value < self.x_axisMinLimit:
                    Value = self.x_axisMinLimit
            elif ofWhatAxis == self.y_axis:
                if Value > self.y_axisMaxLimit:
                    Value = self.y_axisMaxLimit
                if Value < self.y_axisMinLimit:
                    Value = self.y_axisMinLimit
            elif ofWhatAxis == self.z_axis:
                if Value > self.z_axisMaxLimit:
                    Value = self.z_axisMaxLimit
                if Value < self.z_axisMinLimit:
                    Value = self.z_axisMinLimit

            # the accuracy is 0.01
            Value = round(Value,2)
            #write value to PLC
            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = "SetMoveDegree_IMUAPIClass:" + command + "__writeRead fail"
                return -1

            #start to move
            if ofWhatAxis == self.x_axis:
                command = "%01#WCSR002C1"
            elif ofWhatAxis == self.y_axis:
                command = "%01#WCSR002D1"
            elif ofWhatAxis == self.z_axis:
                command = "%01#WCSR002E1"
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SetMoveDegree_IMUAPIClass:" + command + "__writeRead fail"
                return -1

            #wait finsh
            """
            myWaitTime = 0
            while(myWaitTime < timeout):
                time.sleep(0.05)
                ret = self.GetmoveSignal_IMUAPIClass(ofWhatAxis)
                if ret == -1:
                    self.strErrorMessage = "SetMoveDegree_IMUAPIClass:wait finish single time out"
                    return -1
                elif ret == 1:
                    myWaitTime = myWaitTime + 0.05
                else:
                    break
            """

            #set low level
            """
            if ofWhatAxis == self.x_axis:
                command = "%01#WCSR002C0"
            elif ofWhatAxis == self.y_axis:
                command = "%01#WCSR002D0"
            elif ofWhatAxis == self.z_axis:
                command = "%01#WCSR002E0"
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SetMoveDegree_IMUAPIClass:" +  command + "__writeRead fail"
                return -1
            return 0
            """
            return 0
        except:
            self.strErrorMessage = "SetMoveDegree_IMUAPIClass except error"
            return -1


    # **************************************************************#
    #Wait for finishing
    #:ofWhatAxis : x/y/z
    #timeout:the max wait time
    def WaitForSetMoveDegreeFinishing_IMUAPIClass(self, ofWhatAxis, timeout=10):
        myWaitTime = 0
        while(myWaitTime < timeout):
            time.sleep(0.05)
            ret = self.GetmoveSignal_IMUAPIClass(ofWhatAxis)
            if ret == -1:
                self.strErrorMessage = "SetMoveDegree_IMUAPIClass:wait finish single time out"
                return -1
            elif ret == 1:
                myWaitTime = myWaitTime + 0.05
            else:
                break

        if ofWhatAxis == self.x_axis:
            command = "%01#WCSR002C0"
        elif ofWhatAxis == self.y_axis:
            command = "%01#WCSR002D0"
        elif ofWhatAxis == self.z_axis:
            command = "%01#WCSR002E0"
        bcc = self.__bccValue(command)
        command = command + bcc + '\r'
        ret = self.__writeRead(command)

        if (ret != 0):
            self.strErrorMessage = "SetMoveDegree_IMUAPIClass:" +  command + "__writeRead fail"
            return -1

        return 0

    # **************************************************************#
    #Set move degree low level
    #:ofWhatAxis : x/y/z
    #timeout:the max wait time
    def WaitForStepMoveFinishing_IMUAPIClass(self, ofWhatAxis, timeout=10):
        myWaitTime = 0
        while(myWaitTime < timeout):
            time.sleep(0.05)
            ret = self.GetmoveSignal_IMUAPIClass(ofWhatAxis)
            if ret == -1:
                self.strErrorMessage = "SetMoveDegree_IMUAPIClass:wait finish single time out"
                return -1
            elif ret == 1:
                myWaitTime = myWaitTime + 0.05
            else:
                break

        if ofWhatAxis == self.x_axis:
            command = "%01#WCSR00200"
        elif ofWhatAxis == self.y_axis:
            command = "%01#WCSR00240"
        elif ofWhatAxis == self.z_axis:
            command = "%01#WCSR00280"
        bcc = self.__bccValue(command)
        command = command + bcc + '\r'
        ret = self.__writeRead(command)
        if (ret != 0):
            self.strErrorMessage = "SetMoveDegree_IMUAPIClass:" +  command + "__writeRead fail"
            return -1
        return 0

    # **************************************************************#
    #Get degree
    def GetDegree_IMUAPIClass(self,ofWhatAxis):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "GetDegree_IMUAPIClass:the port did not open"
                return -9999

            command = ""
            if ofWhatAxis == self.x_axis:
                command = "%01#RDD0014600147"
            elif ofWhatAxis == self.y_axis:
                command = "%01#RDD0015000151"
            elif ofWhatAxis == self.z_axis:
                command = "%01#RDD0015400155"

            #write command
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            self.myPLCSerialPort.write(command)

            #Convert data
            readString =  self.ReadData(0.05)
            if ("fail" in readString):
                self.strErrorMessage = "GetDegree_IMUAPIClass:ReadData fail"
                return -9999
            else:
                value = self.__getValueOfByte(readString)
                return (value*10)
        except:
            self.strErrorMessage = "GetDegree_IMUAPIClass except error"
            return -9999

    # **************************************************************#
    #go home
    def SignalReset_IMUAPIClass(self,timeout):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "SignalReset_IMUAPIClass:the port did not open"
                return -1
            #clear
            command = '%01#WCSR00840'
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SSignalReset_IMUAPIClass:" +  "clear" + command + "__writeRead fail"
                return -1

            command = '%01#WCSR00841'
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = "SignalReset_IMUAPIClass:" +  command + "__writeRead fail"
                return -1

            #wait finsh
            myWaitTime = 0
            while(myWaitTime < timeout):
                time.sleep(0.2)
                ret = self.GetmoveSignal_IMUAPIClass(self.xyz_axis)
                if ret == -1:
                    self.strErrorMessage = "SignalReset_IMUAPIClass:get xyz move single fail"
                    return -1
                elif ret == 1:
                    myWaitTime = myWaitTime + 0.2
                else:
                    break

            #set low level
            command = '%01#WCSR00840'
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SignalReset_IMUAPIClass:" +  command + "__writeRead fail"
                return -1

            ret = self.SetAxisToLowLevel_IMUAPIClass()
            if (ret != 0):
                return -1
            return 0
        except:
            self.strErrorMessage = "SignalReset_IMUAPIClass except error"
            return -1

    # **************************************************************#
    #get limit
    #ofWhatLimit:max/min
    #ofWhatAxis:x/y/z
    def GetLimit_IMUAPIClass(self,ofWhatLimit,ofWhatAxis):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "GetLimit_IMUAPIClass:the port did not open"
                return -9999
            command = ""
            if ofWhatLimit == self.max_limit:
                if ofWhatAxis == self.x_axis:
                    command = "%01#RDD0062000621"
                elif ofWhatAxis == self.y_axis:
                    command = "%01#RDD0062200623"
                elif ofWhatAxis == self.z_axis:
                    command = "%01#RDD0062400625"
            elif ofWhatLimit == self.min_limit:
                if ofWhatAxis == self.x_axis:
                    command = "%01#RDD0063000631"
                elif ofWhatAxis == self.y_axis:
                    command = "%01#RDD0063200633"
                elif ofWhatAxis == self.z_axis:
                    command = "%01#RDD0063400635"

            #write data
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            self.myPLCSerialPort.write(command)

            #read data
            readString = self.ReadData(0.05)
            if("fail" in readString):
                self.strErrorMessage = "GetLimit_IMUAPIClass:ReadData fail"
                return -9999
            else:
                value = self.__getValueOfByte(readString)
                return (value*10)
        except:
            self.strErrorMessage = "GetLimit__IMUAPIClass except error"
            return -9999

    # **************************************************************#

    # **************************************************************#
    #Set limit
    #ofWhatLimit:max/min
    #ofWhatAxis:x/y/z
    def SetLimit_IMUAPIClass(self,ofWhatLimit,ofWhatAxis,value):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "SetLimit_IMUAPIClass:the port did not open"
                return -9999
            command = ""
            if ofWhatLimit == self.max_limit:
                if ofWhatAxis == self.x_axis:
                    command = "%01#WDD0210002101"
                elif ofWhatAxis == self.y_axis:
                    command = "%01#WDD0210802109"
                elif ofWhatAxis == self.z_axis:
                    command = "%01#WDD0211602117"
            elif ofWhatLimit == self.min_limit:
                if ofWhatAxis == self.x_axis:
                    command = "%01#WDD0210402105"
                elif ofWhatAxis == self.y_axis:
                    command = "%01#WDD0211202113"
                elif ofWhatAxis == self.z_axis:
                    command = "%01#WDD0212002121"

            finalByte = self.__flipByte(value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if ret == 0:
                return 0
            else:
                return -1


        except:
            self.strErrorMessage = "SetLimit__IMUAPIClass except error"
            return -9999

    # **************************************************************#


    #Set Step
    def SetStep_IMUAPIClass(self,ofWhatAxis,Value):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "SetStep_IMUAPIClass:the port did not open"
                return -1
            command = ""
            if ofWhatAxis == self.x_axis:
                command = "%01#WDD0100001001"
            elif ofWhatAxis == self.y_axis:
                command = "%01#WDD0100801009"
            elif ofWhatAxis == self.z_axis:
                command = "%01#WDD0101601017"

            #the accuracy is 0.01
            Value = round(Value,2)
            #write value to PLC
            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = "SetStep_IMUAPIClass:" + command + "__writeRead fail"
                return -1
            return 0
        except:
            self.strErrorMessage = "SetStep_IMUAPIClass except error"
            return -1


    # **************************************************************#
    #Step  move
    def StepMove_IMUAPIClass(self,ofWhatAxis,value,timeout=10):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "StepMove_IMUAPIClass:the port did not open"
                return -1

            #set step
            ret = self.SetStep_IMUAPIClass(ofWhatAxis,value)
            if ret != 0:
                return -1

            #start to move
            command = ""
            if ofWhatAxis == self.x_axis:
                command = "%01#WCSR00201"
            elif ofWhatAxis == self.y_axis:
                command = "%01#WCSR00241"
            elif ofWhatAxis == self.z_axis:
                command = "%01#WCSR00281"
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = "StepMove_IMUAPIClass:" +  command + "__writeRead fail"
                return -1

            #wait finsh
            """
            myWaitTime = 0
            while(myWaitTime < timeout):
                time.sleep(0.05)
                ret = self.GetmoveSignal_IMUAPIClass(ofWhatAxis)
                if ret == -1:
                    self.strErrorMessage = "StepMove_IMUAPIClass:wait finish single time out"
                    return -1
                elif ret == 1:
                    myWaitTime = myWaitTime + 0.05
                else:
                    break
            """


            #set low level
            """
            if ofWhatAxis == self.x_axis:
                command = "%01#WCSR00200"
            elif ofWhatAxis == self.y_axis:
                command = "%01#WCSR00240"
            elif ofWhatAxis == self.z_axis:
                command = "%01#WCSR00280"
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "StepMove_IMUAPIClass:" +  command + "__writeRead fail"
                return -1
            return 0
            """
            return 0
        except:
            self.strErrorMessage = "StepMove_IMUAPIClass except error"
            return -1

    # **************************************************************#
    #Move to put DUT easily
    def EasyToPutDut_IMUAPIClass(self):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "EasyToPutDut_IMUAPIClass:the port did not open"
                return -1
            # ret = self.SetMoveDegree_IMUAPIClass(self.x_axis,30,10)
            # if ret != 0:
            #     return -1
            #
            # ret = self.SetMoveDegree_IMUAPIClass(self.y_axis,90,10)
            # if ret != 0:
            #     return -1
            ret = self.SignalReset_IMUAPIClass(30)
            if ret != 0:
                return -1
        except:
            self.strErrorMessage = "GetLimit__IMUAPIClass except error"
            return -1

    # **************************************************************#
    #Read the state of PLC sensor
    def ReadStateOfPLCSensor_IMUAPIClass(self,sensor):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "ReadStateOfPLCSensor_IMUAPIClass:the port did not open"
                return -1
            command = ""
            if sensor == self.PLCSensorStart:
                command = "%01#RCSX000A"
            elif sensor == self.PLCSensorStop:
                command = "%01#RCSX000B"
            elif sensor == self.PLCSensorReset:
                command = "%01#RCSX000C"
            elif sensor == self.PLCSensorUrgentstop:
                command = "%01#RCSX0009"
            elif sensor == self.PLCSensorCheckDoor:
                command = "%01#RCSR3100"
            elif sensor == self.DUT1SenSor:
                command = "%01#RCSX0014"
            elif sensor == self.DUT2SenSor:
                command = "%01#RCSX0015"
            elif sensor == self.DUT3SenSor:
                command = "%01#RCSX0016"
            elif sensor == self.DUT4SenSor:
                command = "%01#RCSX0017"
            ret = self.__readONorOFF(command)
            ret = int(ret)
            if (ret == -1):
                self.strErrorMessage = "ReadStateOfPLCSensor Read command fail"
                return -1
            return ret
        except:
            self.strErrorMessage = "ReadStateOfSensor except error"
            return -1


    # **************************************************************#
    #Get degree
    def SetAxisToLowLevel_IMUAPIClass(self):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "SetAxisToLowLevel_IMUAPIClass:the port did not open"
                return -1
            for i in range(0,6,1):
                if i == 0:
                    command = "%01#WCSR002C0"
                elif i == 1:
                    command = "%01#WCSR002D0"
                elif i == 2:
                    command = "%01#WCSR002E0"
                elif i == 3:
                    command = "%01#WCSR00240"
                elif i == 4:
                    command = "%01#WCSR00280"
                elif i == 5:
                    command = "%01#WCSR00200"
                bcc = self.__bccValue(command)
                command = command + bcc + '\r'
                ret = self.__writeRead(command)
                if (ret != 0):
                    self.strErrorMessage = "SetAxisToLowLevel_IMUAPIClass:" +  command + "__writeRead fail"
                    return -1
            return 0
        except:
            self.strErrorMessage = "SetAxisToLowLevel_IMUAPIClass except error"
            return -1

    # **************************************************************#
    #Set PLC offset
    def SetPLCOffset_IMUAPIClass(self,ofWhatAxis,Value):
        try:
            if (self.myPLCSerialPort.is_open == False):
                self.strErrorMessage = "SetPLCOffset__IMUAPIClass:the port did not open"
                return -1
            command = ""
            if ofWhatAxis == self.x_axis:
                command = "%01#WDD0114001141"
            elif ofWhatAxis == self.y_axis:
                command = "%01#WDD0113601137"
            elif ofWhatAxis == self.z_axis:
                command = "%01#WDD0114401145"
            #the accuracy is 0.01
            Value = round(Value,2)
            #write value to PLC
            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = "SetPLCOffset__IMUAPIClass:" + command + "__writeRead fail"
                return -1
            return 0
        except:
            self.strErrorMessage = "SetPLCOffset__IMUAPIClass except error"
            return -1

    # **************************************************************#
    #Load PLC offset
    def LoadPLCOffset_IMUAPIClass(self):
        try:
            strPath = os.getcwd() + "/PLCOffset.txt"
            if os.path.exists(strPath) == False:
                self.strErrorMessage = "LoadPLCOffset_IMUAPIClass:can not load PLCOffset.txt"
                return -1
            myOffset = open(strPath,"r")
            while True:
                line = myOffset.readline()
                if len(line) < 2:
                    break
                if "xOffset" in line:
                    xOffset = line[line.find('=')+1:len(line)]
                elif "yOffset" in line:
                    yOffset = line[line.find('=')+1:len(line)]
                elif "zOffset" in line:
                    zOffset = line[line.find('=')+1:len(line)]
            #set value
            ret = self.SetPLCOffset_IMUAPIClass(self.x_axis,float(xOffset))
            if ret != 0:
                return -1
            ret = self.SetPLCOffset_IMUAPIClass(self.y_axis,float(yOffset))
            if ret != 0:
                return -1
            ret = self.SetPLCOffset_IMUAPIClass(self.z_axis,float(zOffset))
            if ret != 0:
                return -1
        except:
            self.strErrorMessage = "LoadPLCOffset_IMUAPIClass except error"
            return -1



    # **************************************************************#
    #read data
    def ReadData(self,timeDelay):
        bReadData = False
        for i in range(0, 5, 1):
            time.sleep(timeDelay)
            readString = self.myPLCSerialPort.readline()
            if(len(readString) > 1):
                return readString
            else:
                continue
        if(bReadData == False):
            return "fail"

    # **************************************************************#
    # Get BCC Value
    def __bccValue(self, code):
        code1 = ord(code[0])
        code2 = ord(code[1])
        bcc = code1 ^ code2
        for i in range(code.__len__() - 2):
            codetem = ord(code[i + 2])
            bcc = bcc ^ codetem
        bcc = binascii.hexlify(struct.pack('>i', bcc))
        bcc = bcc[6:8]
        return bcc


    # **************************************************************#
    # Write and Read Command
    def __writeRead(self, command):
        try:
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            self.myPLCSerialPort.write(command)
            readString = self.ReadData(0.1)
            if (readString[3] == '$'):
                return 0
            else:
                return -1
        except:
            return -1

    # **************************************************************#
    # flip Byte Function
    def __flipByte(self, code):
        code = float(code)
        code = code * 5000.0 / 5.0
        X = binascii.hexlify(struct.pack('>i', code))

        byte1 = X[0:2]
        byte2 = X[2:4]
        byte3 = X[4:6]
        byte4 = X[6:8]
        finalbyte = byte4 + byte3 + byte2 + byte1
        finalbyte = finalbyte.upper()
        return finalbyte



    # **************************************************************#
    # __getValueOfByte function
    def __getValueOfByte(self,ByteString):
        finalbyte = ByteString[6:14]
        byte1 = finalbyte[0:2]
        byte2 = finalbyte[2:4]
        byte3 = finalbyte[4:6]
        byte4 = finalbyte[6:8]
        finalbyte = byte4 + byte3 + byte2 + byte1
        #finalbyte = int(finalbyte, 16)
        #finalbyte = struct.unpack('!i', finalbyte.decode('hex'))[0]
        finalbyte = struct.unpack('!i', binascii.unhexlify(finalbyte))[0]
        finalbyte = float(finalbyte)
        Value = finalbyte * 5.0 / 5000.0
        return Value


    # **************************************************************#
    #__readONorOFF function
    def __readONorOFF(self,command):
        bcc = self.__bccValue(command)
        command = command + bcc + '\r'
        command = command.upper()
        self.myPLCSerialPort.write(command)
        readString = self.ReadData(0.1)#self.ser.readline()
        if("fail" in readString):
            return -1
        readState = readString[6]
        return readState


    # **************************************************************#
    # __readONorOFF function
    def __readVer(self, command):
        bcc = self.__bccValue(command)
        command = command + bcc + '\r'
        command = command.upper()
        self.myPLCSerialPort.write(command)
        readString = self.ReadData(0.1)  # self.ser.readline()
        if ("fail" in readString):
            return -1
        readState1 = readString[6:8]
        readState2 = readString[8:10]
        s1 = binascii.a2b_hex(readState1)
        s2 = binascii.a2b_hex(readState2)
        ver = s1 + '.' + s2
        return ver




    #the below code is for sensor
    # **************************************************************#
    def SensorSetToIdel_IMUAPIClass(self):
        try:
            #laod default setting and set to idle
            bopenSWStepOne = False
            for i in range(0,5,1):
                self.mySensorSerialPort.write(self.openSWStepOne)
                time.sleep(0.1)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                print("openSWStepOne = " +  myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3] != "7565":
                    continue
                else:
                    bopenSWStepOne = True
                    break
            if bopenSWStepOne == False:
                self.strErrorMessage = "SensorInitial_IMUAPIClass:openSWStepOne fail"
                return -1




            bopenSWStepTwo = False
            for i in range(0,5,1):
                self.mySensorSerialPort.write(self.openSWStepTwo)
                time.sleep(0.1)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                print("openSWStepTwo = " + myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3] != "7565":
                    continue
                else:
                    bopenSWStepTwo = True
                    break
            if bopenSWStepTwo == False:
                self.strErrorMessage = "SensorInitial_IMUAPIClass:openSWStepTwo fail"
                return -1

            bopenSWStepThree = False
            for i in range(0,5,1):
                self.mySensorSerialPort.write(self.openSWStepThree)
                time.sleep(0.1)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                print("openSWStepThree = " + myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3] != "7565":
                    continue
                else:
                    bopenSWStepThree = True
                    break
            if bopenSWStepThree == False:
                self.strErrorMessage = "SensorInitial_IMUAPIClass:openSWStepThree fail"
                return -1

            return 0
        except:
            self.strErrorMessage = "OpenSensorSetToIdel_IMUAPIClass except error"
            return -1

    # **************************************************************#
    #Sensor set to Euler mode
    def SensorSetEulerRPY_IMUAPIClass(self):
        try:
            if(self.mySensorSerialPort.is_open == False):
                self.strErrorMessage = "the sensor port did not open"
                return -1

            #step 1:
            bStepOne= False
            for i in range(0, 5, 1):
                self.mySensorSerialPort.write(self.stepOne)
                time.sleep(0.1)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                print("Step 1 = " + myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3] != "7565":
                    continue
                else:
                    bStepOne = True
                    break
            if bStepOne == False:
                self.strErrorMessage = "SensorInitial_IMUAPIClass:initial step 1 fail"
                return -1


            # step 2:
            bStepTwo= False
            for i in range(0, 5, 1):
                self.mySensorSerialPort.write(self.stepTwo)
                time.sleep(0.1)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                print("Step 2 = " + myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3] != "7565":
                    continue
                else:
                    bStepTwo = True
                    break
            if bStepTwo == False:
                self.strErrorMessage = "SensorInitial_IMUAPIClass:initial step 2 fail"
                return -1



            # step 3:
            bStepThree= False
            for i in range(0, 5, 1):
                self.mySensorSerialPort.write(self.stepThree)
                time.sleep(0.1)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                print("Step 3 = " + myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3] != "7565":
                    continue
                else:
                    bStepThree = True
                    break
            if bStepThree == False:
                self.strErrorMessage = "SensorInitial_IMUAPIClass:initial step 3 fail"
                return -1


            # step 4:
            bStepFour= False
            for i in range(0, 5, 1):
                self.mySensorSerialPort.write(self.stepFour)
                time.sleep(0.1)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                print("Step 4 = " + myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3] != "7565":
                    continue
                else:
                    bStepFour = True
                    break
            if bStepFour == False:
                self.strErrorMessage = "SensorInitial_IMUAPIClass:initial step 4 fail"
                return -1



            # step startAcquire:
            """
            bStepstartAcquire = False
            for i in range(0,5,1):
                self.mySensorSerialPort.write(self.startAcquire)
                time.sleep(0.1)
                #strRead = self.mySensorSerialPort.read(20)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                print("Step startAcquire = " + myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3] != "7565":
                    continue
                else:
                    bStepstartAcquire = True
                    break
            if bStepstartAcquire == False:
                self.strErrorMessage = "SensorInitial_IMUAPIClass:initial step startAcquire fail"
                return -1
            """

            # # step stopAcquire:
            # bStepstopAcquire = False
            # for i in range(0,5,1):
            #     self.mySensorSerialPort.write(self.stopAcquire)
            #     time.sleep(0.1)
            #     #strRead = self.mySensorSerialPort.read(20)
            #     strRead = self.mySensorSerialPort.read_all()
            #     myarray = []
            #     myarray = binascii.b2a_hex(strRead)
            #     print("Step stopAcquire = " + myarray)
            #     if myarray[0]+ myarray[1] + myarray[2] + myarray[3] != "7565":
            #         continue
            #     else:
            #         bStepstopAcquire = True
            #         break
            # if bStepstopAcquire == False:
            #     self.strErrorMessage = "SensorInitial_IMUAPIClass:initial step stopAcquire fail"
            #     return -1
            # #self.mySensorSerialPort.close()
            return 0
        except:
            self.strErrorMessage = "SensorInitial_IMUAPIClass except error"
            return -1

    # **************************************************************#
    #Sensor clear default setting
    def SensorClearSetting_IMUAPIClass(self):
        try:
            if(self.mySensorSerialPort.is_open == False):
                self.OpenPort_IMUAPIClass(self.mySensorSerialPortName, self.SwitchSensor)
                self.mySensorSerialPort.read_all()
                self.mySensorSerialPort.flushOutput()

            #step 3
            bClearSettingThree= False
            for i in range(0, 5, 1):
                self.mySensorSerialPort.write(self.clearSettingThree)
                time.sleep(0.1)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                print("bClearSettingThree = " + myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3] != "7565":
                    continue
                else:
                    bClearSettingThree = True
                    break
            if bClearSettingThree == False:
                self.strErrorMessage = "SensorClearSetting_IMUAPIClass:initial step 3 fail"
                return -1

            #step 4
            bClearSettingFour= False
            for i in range(0, 5, 1):
                self.mySensorSerialPort.write(self.clearSettingFour)
                time.sleep(0.1)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                print("bClearSettingFour = " + myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3] != "7565":
                    continue
                else:
                    bClearSettingFour = True
                    break
            if bClearSettingFour == False:
                self.strErrorMessage = "SensorClearSetting_IMUAPIClass:initial step 4 fail"
                return -1
            return 0
        except:
            self.strErrorMessage = "SensorClearSetting_IMUAPIClass except error"
            return -1

    # **************************************************************#
    #Measure degree from sensor
    def SensorGetDegreeFromSensor_IMUAPIClass(self,HZ,Readtime,timeInterval,ref_readings,average):
        try:
            if(self.mySensorSerialPort.is_open == False):
                self.OpenPort_IMUAPIClass(self.mySensorSerialPortName, self.SwitchSensor)
                self.mySensorSerialPort.read_all()
                self.mySensorSerialPort.flushOutput()

            TotalRead = int(HZ * Readtime)
            for i in range(0,TotalRead,1):
                ReRead = 0
                while(ReRead < 5):
                    self.mySensorSerialPort.write(self.singleGetData)
                    time.sleep(timeInterval)
                    strRead = self.mySensorSerialPort.read_all()
                    myarray = []
                    myarray = binascii.b2a_hex(strRead)
                    if myarray[0] + myarray[1] + myarray[2] + myarray[3] == "7565":
                        break
                    else:
                        ReRead += 1
                if ReRead >= 5:
                    self.strErrorMessage = "SensorGetDegreeFromSensor_IMUAPIClass:Read data error"
                    return -1

                #covert data to float
                dataList = [0, 0, 0]
                ret = self.SensorConverStrDataToFloat_IMUAPIClass(myarray,dataList)
                ref_readings.append(dataList)

            floatSumRoll = 0.0
            floatSumPitch = 0.0
            floatSumYaw = 0.0
            for i in range(0,len(ref_readings),1):
                # print(ref_readings[i])
                floatSumRoll += ref_readings[i][0]
                floatSumPitch += ref_readings[i][1]
                floatSumYaw += ref_readings[i][2]

            #get average of list
            #average = [0, 0, 0]
            #average[0] = round(floatSumRoll / TotalRead,3)
            #average[1] = round(floatSumPitch / TotalRead,3)
            #average[2] =  round(floatSumYaw / TotalRead,3)
            average.append(round(floatSumRoll / TotalRead,3))
            average.append(round(floatSumPitch / TotalRead,3))
            average.append(round(floatSumYaw / TotalRead,3))

            #offset PLC


            if(average[0] < -180 or average[0] > 180):
                average[0] = 180

            if(average[1] < -90):
                average[1] = -90
            if (average[1] > 90):
                average[1] = 90

            return 0
        except:
            self.strErrorMessage = "SensorGetDegreeFromSensor_IMUAPIClass except error"
            return 1



    # **************************************************************#
    #Converet data
    def SensorConverStrDataToFloat_IMUAPIClass(self,sourlist,dataList):
        try:
            strSourcelist = str(sourlist)
            strRoll = strSourcelist[12:20]
            strPitch = strSourcelist[20:28]
            strYaw = strSourcelist[28:36]
            floatRoll = struct.unpack('!f', strRoll.decode('hex'))[0] * 57.5
            floatPitch = struct.unpack('!f', strPitch.decode('hex'))[0] * 57.5
            floatYaw = struct.unpack('!f', strYaw.decode('hex'))[0] * 57.5
            dataList[0] = round(floatRoll,4)
            dataList[1] = round(floatPitch,4)
            dataList[2] = round(floatYaw,4)
            return 0
        except:
            self.strErrorMessage = "ConverStrDataToFloat_IMUAPIClass except error"
            return -1


    # **************************************************************#
    #Sensor offset PLC
    #ofWhatAxis : x/y/z
    #degree : the degree you want to move
    #offsetMaxTimes : the offset max times
    #accuracy : margin
    def SensorOffsetPLC_IMUAPIClass(self,ofWhatAxis,degree,offsetMaxTime,accuracy):
        try:
            #get current time
            start = time.time()
            bOffsetTrue = False
            #calibrate x axis
            if ofWhatAxis == self.x_axis:
                if degree <= -180:
                    degree = -180
                elif degree >= 180:
                    degree = 180

                #********************************************************#
                #if best positon is ok, just load
                if degree >= 0:
                    if self.XAxisBest_0 != 0 and degree == 0:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis,self.XAxisBest_0,10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                        if ret != 0:
                            return -1
                    elif self.XAxisBest_180 != 0 and degree == 180:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis,self.XAxisBest_180,10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                        if ret != 0:
                            return -1
                    elif self.XAxisBest_90_Plus != 0 and degree == 90:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis, self.XAxisBest_90_Plus, 10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                        if ret != 0:
                            return -1
                    else:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis,180-degree,10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                        if ret != 0:
                            return -1
                else:
                    if self.XAxisBest_90_Minus != 0 and degree == -90:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis, self.XAxisBest_90_Minus, 10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                        if ret != 0:
                            return -1
                    else:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis,abs(degree)-180,10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                        if ret != 0:
                            return -1
                # ********************************************************#


                # 2:move
                while ((time.time() - start) < offsetMaxTime):
                    ref_readings = []
                    average = []
                    ret = self.SensorGetDegreeFromSensor_IMUAPIClass(50,0.1,0.01,ref_readings,average)
                    print("Degree from sensor: ", average)
                    if ret != 0:
                        return -1

                    #offset sensor
                    average[0] = self.SensorOffset_IMUAPIClass(average[0],ofWhatAxis,degree)
                    print("Degree with offset: ", average)
                    if ret == -9999:
                        return -1

                    tolerance = abs(abs(average[0]) - abs(degree))
                    if tolerance >= 0 and tolerance <= abs(accuracy):
                        print("Finish x axis offset,tolerance=%f"%(tolerance))
                        bOffsetTrue = True
                        break
                    #get offset degree
                    if degree == 180:
                        if average[0] > 0:
                            offset = average[0] - degree
                        else:
                            offset = degree + average[0]
                    else:
                        offset = average[0] - degree
                    print("degree=%f average[0]=%f offset=%f" %(degree,average[0],offset))
                    ret = self.StepMove_IMUAPIClass(self.x_axis, offset, 10)
                    ret = self.WaitForStepMoveFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                    if ret != 0:
                        return -1

                # save best coordinate
                if bOffsetTrue == True:
                    bestCoordinate = self.GetDegree_IMUAPIClass(self.x_axis)
                    print("Best coordinate: ", bestCoordinate)
                    if bestCoordinate == -9999:
                        return -1
                    if degree == 0:
                        self.XAxisBest_0 = bestCoordinate
                    elif degree == 180:
                        self.XAxisBest_180 = bestCoordinate
                    elif degree == 90:
                        self.XAxisBest_90_Plus = bestCoordinate
                    elif degree == -90:
                        self.XAxisBest_90_Minus = bestCoordinate


            #calibrate y axis
            elif ofWhatAxis == self.y_axis:
                if degree != 0:
                    if degree < -90:
                        degree = -90
                    elif degree > 90:
                        degree = 90

                    #offset
                    # if degree == 90 or degree == -90:
                    #     if degree == 90:
                    #         ret = self.SetMoveDegree_IMUAPIClass(self.x_axis, 1.6, 10)
                    #     else:
                    #         ret = self.SetMoveDegree_IMUAPIClass(self.x_axis, 1.1, 10)
                    #     if ret != 0:
                    #         return 1

                    #move rough y first
                    if degree == 90 and self.YAxisBest_90_Plus != 0:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis,self.YAxisBest_90_Plus, 10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                    elif degree == -90 and self.YAxisBest_90_Minus != 0:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis,self.YAxisBest_90_Minus, 10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                    elif(degree == 90 or degree == -90):
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis,degree-2, 10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                    else:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis, degree/2, 10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                    if ret != 0:
                        return -1


                    minTolerance = 360
                    minTolerancePLCCoordinate = 0
                    step = 0.5
                    while ((time.time() - start) < offsetMaxTime):
                        ref_readings = []
                        average = []
                        ret = self.SensorGetDegreeFromSensor_IMUAPIClass(50, 0.1, 0.01, ref_readings, average)
                        print("Degree from sensor: ", average)
                        if ret != 0:
                            return 1

                        # offset sensor
                        average[1] = self.SensorOffset_IMUAPIClass(average[1], ofWhatAxis, degree)
                        print("Degree with offset: ", average)
                        if ret == -9999:
                            return -1

                        tolerance = abs(abs(average[1]) - abs(degree))
                        print("degree=%f average[1]=%f tolerance=%f" % (degree, average[1], tolerance))
                        if tolerance >= 0 and tolerance <= abs(accuracy):
                            print("Finish y axis offset,tolerance=%f"%(tolerance))
                            bOffsetTrue = True
                            break
                        elif (tolerance < minTolerance):
                            minTolerance = tolerance
                            minTolerancePLCCoordinate = self.GetDegree_IMUAPIClass(self.y_axis)
                            if minTolerancePLCCoordinate == -9999:
                                return -1
                        else:
                            ret = self.SetMoveDegree_IMUAPIClass(self.y_axis,minTolerancePLCCoordinate-step/2,10)
                            ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis,10) + ret
                            ref_readings = []
                            average = []
                            ret = self.SensorGetDegreeFromSensor_IMUAPIClass(50, 0.1, 0.01, ref_readings, average)
                            if ret != 0:
                                return 1

                            # offset sensor
                            average[1] = self.SensorOffset_IMUAPIClass(average[1], ofWhatAxis, degree)
                            if ret == -9999:
                                return -1


                            tolerance = abs(abs(average[1]) - abs(degree))
                            if tolerance >= 0 and tolerance <= abs(accuracy):
                                print("Finish y axis offset,tolerance=%f" % (tolerance))
                                bOffsetTrue = True
                                break
                            if tolerance >= 2:
                                divideFactor = 5
                            else:
                                divideFactor = 10

                            step = tolerance / divideFactor
                            if step < 0.01:
                                step = 0.01

                        #step move
                        ret = self.StepMove_IMUAPIClass(self.y_axis, step, 10)
                        ret = self.WaitForStepMoveFinishing_IMUAPIClass(ofWhatAxis, 10) + ret
                        if ret != 0:
                            return -1
                else:
                    # move rough y first
                    if degree == 0 and self.YAxisBest_0 != 0:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis, self.YAxisBest_0, 10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis, 10) + ret
                    else:
                        ret = self.SetMoveDegree_IMUAPIClass(ofWhatAxis, degree, 10)
                        ret = self.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis, 10) + ret
                    if ret != 0:
                        return -1

                    while ((time.time() - start) < offsetMaxTime):
                        ref_readings = []
                        average = []
                        ret = self.SensorGetDegreeFromSensor_IMUAPIClass(50, 0.1, 0.01, ref_readings, average)
                        print("Degree from sensor: ", average)
                        if ret != 0:
                            return -1

                        # offset sensor
                        average[1] = self.SensorOffset_IMUAPIClass(average[1], ofWhatAxis, degree)
                        print("Degree with offset: ", average)
                        if ret == -9999:
                            return -1


                        tolerance = abs(abs(average[1]) - abs(degree))
                        if tolerance >= 0 and tolerance <= abs(accuracy):
                            bOffsetTrue = True
                            print("Finish y axis offset,tolerance=%f" % (tolerance))
                            break

                        offset = degree - average[1]
                        ret = self.StepMove_IMUAPIClass(self.y_axis, offset, 10)
                        ret = self.WaitForStepMoveFinishing_IMUAPIClass(ofWhatAxis, 10) + ret
                        print("degree=%f average[1]=%f offset=%f" % (degree, average[1], offset))
                        if ret != 0:
                            return -1

                # save best coordinate
                if bOffsetTrue == True:
                    bestCoordinate = self.GetDegree_IMUAPIClass(self.y_axis)
                    if bestCoordinate == -9999:
                        return -1
                    if degree == 90:
                        self.YAxisBest_90_Plus = bestCoordinate
                    elif degree == -90:
                        self.YAxisBest_90_Minus = bestCoordinate
                    elif degree == 0:
                        self.YAxisBest_0 = bestCoordinate

            elif ofWhatAxis == self.z_axis:
                self.strErrorMessage = "it does not support calibrate z-axis"
                return -1


            if bOffsetTrue == False:
                self.strErrorMessage = "offset timeout"
                return -1

            return 0
        except:
            self.strErrorMessage = "SensorOffsetPLC_IMUAPIClass except error"
            return -1



    # **************************************************************#
    #Load Sensor offset
    def LoadSensorOffset_IMUAPIClass(self):
        try:
            strPath = os.getcwd() + "/SensorOffset.txt"
            if os.path.exists(strPath) == False:
                self.strErrorMessage = "LoadSensorOffset_IMUAPIClass:can not find SensorOffset.txt"
                return -1
            mySensorOffset = open(strPath,"r")
            while True:
                line = mySensorOffset.readline()
                if len(line) < 3:
                    break
                if "xAxis0" in line:
                    strData = line[line.find('=')+1:len(line)]
                    self.SensorOffset[0] = float(strData)
                elif "xAxis180" in line:
                    strData = line[line.find('=') + 1:len(line)]
                    self.SensorOffset[1] = float(strData)
                elif "xAxis90Plus" in line:
                    strData = line[line.find('=') + 1:len(line)]
                    self.SensorOffset[2] = float(strData)
                elif "xAxis90Minus" in line:
                    strData = line[line.find('=') + 1:len(line)]
                    self.SensorOffset[3] = float(strData)
                elif "yAxis90Plus" in line:
                    strData = line[line.find('=') + 1:len(line)]
                    self.SensorOffset[4] = float(strData)
                elif "yAxis90Minus" in line:
                    strData = line[line.find('=') + 1:len(line)]
                    self.SensorOffset[5] = float(strData)
            mySensorOffset.close()
            return 0
        except:
            self.strErrorMessage = "LoadSensorOffset__IMUAPIClass except error"
            return -1


    # **************************************************************#
    #Sensor offset
    def SensorOffset_IMUAPIClass(self,sensorDegree,ofWhatAxis,PLCDegree):
        try:
            if ofWhatAxis == self.x_axis:
                if PLCDegree == 0:
                    sensorDegree = sensorDegree + self.SensorOffset[0]
                elif PLCDegree == 180:
                    sensorDegree = sensorDegree + self.SensorOffset[1]
                elif PLCDegree == 90:
                    sensorDegree = sensorDegree + self.SensorOffset[2]
                elif PLCDegree == -90:
                    sensorDegree = sensorDegree + self.SensorOffset[3]
            elif ofWhatAxis == self.y_axis:
                if PLCDegree == 90:
                    sensorDegree = sensorDegree + self.SensorOffset[4]
                elif PLCDegree == -90:
                    sensorDegree = sensorDegree + self.SensorOffset[5]
            return sensorDegree
        except:
            self.strErrorMessage = "SensorOffset_IMUAPIClass except error"
            return -9999

    def SensorGetAccelerationAndAnglesRate_IMUAPIClass(self, accelerationDataList, anglesRateDataList):
        try:
            for i in range(0,5,1):
                self.mySensorSerialPort.write(self.pollData)
                time.sleep(0.02)
                strRead = self.mySensorSerialPort.read_all()
                myarray = []
                myarray = binascii.b2a_hex(strRead)
                #print("Poll data = " + myarray)
                if myarray[0] + myarray[1] + myarray[2] + myarray[3]+ myarray[4] + myarray[5] == "756580":
                    break

            sourceData = myarray.decode()
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

            return 0
        except IOError:
            self.strErrorMessage = "PollingData_IMUAPIClass except error"
            return -1
    # **************************************************************#
    def SetOriginalPosition(self):
        try:
            command_on = '%01#WCSR00441'
            ret = self.__writeRead(command_on)
            if ret != 0:
                self.strErrorMessage = "SetPosition_IMUAPIClass:" +  command_on + "__writeRead fail"
                return -1
            time.sleep(0.2)

            command_off = "%01#WCSR00440"
            ret = self.__writeRead(command_off)
            if ret != 0:
                self.strErrorMessage = "SetPosition_IMUAPIClass:" +  command_off + "__writeRead fail"
                return -1
            return 0
        except:
            self.strErrorMessage = "SetPosition_IMUAPIClass except error"
            return -1


if __name__ == '__main__':
    api = BojayIMUAPI()
    api.OpenPort_IMUAPIClass('COM12', 1)
    api.SensorSetToIdel_IMUAPIClass()
    api.SensorSetEulerRPY_IMUAPIClass()
    ref=[]
    avg=[]
    api.SensorGetDegreeFromSensor_IMUAPIClass(2,2,1,ref,avg)
    acc=[]
    ang=[]
    api.SensorGetAccelerationAndAnglesRate_IMUAPIClass(acc,ang)
    api.ClosePort_IMUAPIClass(1)
