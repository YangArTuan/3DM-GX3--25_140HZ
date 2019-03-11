from PySide2 import QtCore,QtGui,QtWidgets
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QDialog,QMessageBox
import sys
import os
from UI import *
from BojayIMUAPI import *



myBojayIMUAPI = BojayIMUAPI()


class TonyFrame(QDialog):

    # **************************************************************#
    #initial function
    def __init__(self, parent=None):
        try:
            super(TonyFrame,self).__init__(parent)
            self.ui = Ui_Form()
            self.ui.setupUi(self)

            #pushButtonOpenPort
            self.ui.pushButtonOpenPort.clicked.connect(self.OpenPort)

            # pushButtonClosePort
            self.ui.pushButtonClosePort.clicked.connect(self.ClosePort)

            #pushButtonSpeedSet
            self.ui.pushButtonSpeedSet.clicked.connect(self.SetAxisSpeed)

            #pushButtonSpeedGet
            self.ui.pushButtonSpeedGet.clicked.connect(self.GetAxisSpeed)

            #pushButtonLimitGet
            self.ui.pushButtonLimitGet.clicked.connect(self.GetAxisLimit)

            #pushButtonLimitSet
            self.ui.pushButtonSetLimit.clicked.connect(self.SetAxisLimit)

            #pushButtonMovex
            self.ui.pushButtonMovex.clicked.connect(lambda :self.MoveAxisCoordinate(myBojayIMUAPI.x_axis))

            #pushButtonMovey
            self.ui.pushButtonMovey.clicked.connect(lambda :self.MoveAxisCoordinate(myBojayIMUAPI.y_axis))

            #pushButtonMovez
            self.ui.pushButtonMovez.clicked.connect(lambda :self.MoveAxisCoordinate(myBojayIMUAPI.z_axis))

            #pushButtonGetx
            self.ui.pushButtonGetx.clicked.connect(lambda :self.GetcurrentCoordinate(myBojayIMUAPI.x_axis))

            #pushButtonGety
            self.ui.pushButtonGety.clicked.connect(lambda :self.GetcurrentCoordinate(myBojayIMUAPI.y_axis))

            #pushButtonGetz
            self.ui.pushButtonGetz.clicked.connect(lambda :self.GetcurrentCoordinate(myBojayIMUAPI.z_axis))

            #pushButtonOffsetX
            self.ui.pushButtonOffsetX.clicked.connect(lambda: self.OffsetPLC(myBojayIMUAPI.x_axis))

            #pushButtonOffsetY
            self.ui.pushButtonOffsetY.clicked.connect(lambda: self.OffsetPLC(myBojayIMUAPI.y_axis))

            #pushButtonOffsetZ
            self.ui.pushButtonOffsetZ.clicked.connect(lambda: self.OffsetPLC(myBojayIMUAPI.z_axis))

            #pushButtonGetSensorData
            self.ui.pushButtonGetSensorData.clicked.connect(self.UpdateSensorData)

            #ButtonAcquire
            self.ui.ButtonAcquire.clicked.connect(self.AcquireSensorDataTable)

            #AcquireAcceleration
            self.ui.AcquireAcceleration.clicked.connect(self.AcquireSensorAccelerationAndAnglesRate)

            #password
            self.ui.pushButtonSetPosition.clicked.connect(self.SetPositionfunction)
            
            self.ui.pushButtonClosePort.setEnabled(False)
            self.SwitchControlsState(False)



            #ButtonStepMoveX
            self.ui.ButtonStepMoveX.clicked.connect(lambda: self.StepMove(myBojayIMUAPI.x_axis))
            #ButtonStepMoveY
            self.ui.ButtonStepMoveY.clicked.connect(lambda: self.StepMove(myBojayIMUAPI.y_axis))
            #ButtonStepMoveZ
            self.ui.ButtonStepMoveZ.clicked.connect(lambda: self.StepMove(myBojayIMUAPI.z_axis))
            #ButtonBurning
            self.ui.ButtonBurning.clicked.connect(self.Burning)


            #pushButtonSetPLCOffset
            self.ui.pushButtonSetPLCOffset.clicked.connect(self.SetPLCOffset)


            #Set initial value for offset accuracy
            self.ui.textEditxAccuracyX.setText("0.1")
            self.ui.textEditxAccuracyY.setText("0.1")
            self.ui.textEditxAccuracyZ.setText("0.1")

            self.ui.textEditXStep.setText("0.1")
            self.ui.textEditYStep.setText("0.1")
            self.ui.textEditZStep.setText("0.1")

        except:
            self.ShowErroeMessage("__init__  except Fai")


    # **************************************************************#
    # show error message
    def ShowErroeMessage(self, message):
        try:
            myMessageBox = QMessageBox()
            myMessageBox.information(self, "Warning", message, myMessageBox.Ok)
            return 0
        except:
            self.ShowErroeMessage("ShowErroeMessage except Fail")
            return -1


    # **************************************************************#
    # switch controls state
    def SwitchControlsState(self,state):
        try:
            print("you are running SwitchControlsState")

            return 0
        except:
            self.ShowErroeMessage("SwitchControlsState except Fail")
            return -1


    # **************************************************************#
    # Open the port
    def OpenPort(self):
        try:
            myBojayIMUAPI.LoadSensorOffset_IMUAPIClass()
            #return
            if self.ui.checkBoxPLC.isChecked() == True:
                ret = myBojayIMUAPI.OpenPort_IMUAPIClass("/dev/ttyUSB3",myBojayIMUAPI.SwitchPLC)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
            if self.ui.checkBoxSensor.isChecked() == True:

                #open first
                ret = myBojayIMUAPI.OpenPort_IMUAPIClass("/dev/ttyUSB2",myBojayIMUAPI.SwitchSensor)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
                #clear setting
                ret = myBojayIMUAPI.SensorClearSetting_IMUAPIClass()
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

                #set to idle
                ret = myBojayIMUAPI.SensorSetToIdel_IMUAPIClass()
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

                ret = myBojayIMUAPI.SensorSetEulerRPY_IMUAPIClass()
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
            self.ui.pushButtonClosePort.setEnabled(True)
            self.SwitchControlsState(True)
            self.ui.pushButtonOpenPort.setEnabled(False)

            # myBojayIMUAPI.SensorSetToIdel_IMUAPIClass()
            # myBojayIMUAPI.SensorSetEulerRPY_IMUAPIClass()
            return 0
        except:
            self.ShowErroeMessage("OpenPort except Fail")
            return -1


    # **************************************************************#
    # Close the port
    def ClosePort(self):
        try:
            print ("you are running ClosePort function")
            if self.ui.checkBoxPLC.isChecked() == True:
                ret = myBojayIMUAPI.ClosePort_IMUAPIClass(0)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

            if self.ui.checkBoxSensor.isChecked() == True:
                ret = myBojayIMUAPI.ClosePort_IMUAPIClass(1)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

            self.ui.pushButtonClosePort.setEnabled(False)
            self.SwitchControlsState(False)
            self.ui.pushButtonOpenPort.setEnabled(True)
            return 0
        except:
            self.ShowErroeMessage("ClosePort except Fail")
            return -1


    # **************************************************************#
    # Set axis speed
    def SetAxisSpeed(self):
        try:

            #check x axis
            xSpeed = self.ui.textEditXSpeedSet.toPlainText()
            result = self.CheckEditBox(True,xSpeed)
            if(result != 0):
                return -1

            ret = myBojayIMUAPI.SetAxisSpeed_IMUAPIClass(myBojayIMUAPI.x_axis,int(xSpeed))
            if ret != 0:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1

            #check y axis
            ySpeed = self.ui.textEditYSpeedSet.toPlainText()
            result = self.CheckEditBox(True,ySpeed)
            if(result != 0):
                return
            ret = myBojayIMUAPI.SetAxisSpeed_IMUAPIClass(myBojayIMUAPI.y_axis,int(ySpeed))
            if ret != 0:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1

            #check z axis
            zSpeed = self.ui.textEditZSpeedSet.toPlainText()
            result = self.CheckEditBox(True,zSpeed)
            if(result != 0):
                return
            ret = myBojayIMUAPI.SetAxisSpeed_IMUAPIClass(myBojayIMUAPI.z_axis,int(zSpeed))
            if ret != 0:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1

            return 0
        except:
            self.ShowErroeMessage("SetAxisSpeed except Fail")
            return -1


    # **************************************************************#
    # Get axis speed
    def GetAxisSpeed(self):
        try:
            #check x axis
            ret = myBojayIMUAPI.GetAxisSpeed_IMUAPIClass(myBojayIMUAPI.x_axis)
            if(ret == -1):
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
            self.ui.textEditXSpeedSet.setText(str(ret))


            #check y axis
            ret = myBojayIMUAPI.GetAxisSpeed_IMUAPIClass(myBojayIMUAPI.y_axis)
            if(ret == -1):
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
            self.ui.textEditYSpeedSet.setText(str(ret))


            #check z axis
            ret = myBojayIMUAPI.GetAxisSpeed_IMUAPIClass(myBojayIMUAPI.z_axis)
            if(ret == -1):
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
            self.ui.textEditZSpeedSet.setText(str(ret))
            return 0
        except:
            self.ShowErroeMessage("GetAxisSpeed except Fail")
            return -1

    # Get limit
    def GetAxisLimit(self):
        try:
            #get x max limit
            ret = myBojayIMUAPI.GetLimit_IMUAPIClass(myBojayIMUAPI.max_limit,myBojayIMUAPI.x_axis)
            if ret == -9999:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
            self.ui.xMaxLimit.setText(str(ret))

            # get x min limit
            ret = myBojayIMUAPI.GetLimit_IMUAPIClass(myBojayIMUAPI.min_limit,myBojayIMUAPI.x_axis)
            if ret == -9999:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
            self.ui.xMinLimit.setText(str(ret))


            #get y max limit
            ret = myBojayIMUAPI.GetLimit_IMUAPIClass(myBojayIMUAPI.max_limit,myBojayIMUAPI.y_axis)
            if ret == -9999:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
            self.ui.yMaxLimit.setText(str(ret))

            # get y min limit
            ret = myBojayIMUAPI.GetLimit_IMUAPIClass(myBojayIMUAPI.min_limit,myBojayIMUAPI.y_axis)
            if ret == -9999:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
            self.ui.yMinLimit.setText(str(ret))



            #get z max limit
            ret = myBojayIMUAPI.GetLimit_IMUAPIClass(myBojayIMUAPI.max_limit,myBojayIMUAPI.z_axis)
            if ret == -9999:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
            self.ui.zMaxLimit.setText(str(ret))

            # get z min limit
            ret = myBojayIMUAPI.GetLimit_IMUAPIClass(myBojayIMUAPI.min_limit,myBojayIMUAPI.z_axis)
            if ret == -9999:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
            self.ui.zMinLimit.setText(str(ret))

        except:
            self.ShowErroeMessage("GetAxisLimit except Fail")
            return -1


    # **************************************************************#
    # Move to dest
    def MoveAxisCoordinate(self,ofWhatAxis):
        try:
            # #CheckFourDUTSensor
            # ret = myBojayIMUAPI.ReadStateOfPLCSensor_IMUAPIClass(myBojayIMUAPI.DUT1SenSor)
            # if(ret!=1):
            #     self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
            #     return -1
            # ret = myBojayIMUAPI.ReadStateOfPLCSensor_IMUAPIClass(myBojayIMUAPI.DUT2SenSor)
            # if(ret!=1):
            #     self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
            #     return -1
            # ret = myBojayIMUAPI.ReadStateOfPLCSensor_IMUAPIClass(myBojayIMUAPI.DUT3SenSor)
            # if(ret!=1):
            #     self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
            #     return -1
            # ret = myBojayIMUAPI.ReadStateOfPLCSensor_IMUAPIClass(myBojayIMUAPI.DUT4SenSor)
            # if(ret!=1):
            #     self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
            #     return -1

            print(ofWhatAxis)
            if(ofWhatAxis == myBojayIMUAPI.x_axis):
                xValue = self.ui.textEditxMove.toPlainText()
                ret = self.CheckEditBox(True, xValue)
                if ret != 0:
                    return -1
                ret = myBojayIMUAPI.SetMoveDegree_IMUAPIClass(myBojayIMUAPI.x_axis,float(xValue),10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
                ret = myBojayIMUAPI.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis, 10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
            elif(ofWhatAxis == myBojayIMUAPI.y_axis):
                yValue = self.ui.textEdityMove.toPlainText()
                ret = self.CheckEditBox(True, yValue)
                if ret != 0:
                    return -1
                ret = myBojayIMUAPI.SetMoveDegree_IMUAPIClass(myBojayIMUAPI.y_axis,float(yValue),10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
                ret = myBojayIMUAPI.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis, 10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
            elif(ofWhatAxis == myBojayIMUAPI.z_axis):
                zValue = self.ui.textEditzMove.toPlainText()
                ret = self.CheckEditBox(True, zValue)
                if ret != 0:
                    return -1
                ret = myBojayIMUAPI.SetMoveDegree_IMUAPIClass(myBojayIMUAPI.z_axis,float(zValue),10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
                ret = myBojayIMUAPI.WaitForSetMoveDegreeFinishing_IMUAPIClass(ofWhatAxis, 10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
        except:
            self.ShowErroeMessage("MoveAxisCoordinate except Fail")
            return -1

    # **************************************************************#
    # Get current coordinate
    def GetcurrentCoordinate(self,ofWhatAxis):
        try:
            #get x axis
            if (ofWhatAxis == myBojayIMUAPI.x_axis):
                ret = myBojayIMUAPI.GetDegree_IMUAPIClass(myBojayIMUAPI.x_axis)
                if ret == -9999:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
                self.ui.textEditxMove.setText(str(ret))


            # get y axis
            elif (ofWhatAxis == myBojayIMUAPI.y_axis):
                ret = myBojayIMUAPI.GetDegree_IMUAPIClass(myBojayIMUAPI.y_axis)
                if ret == -9999:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
                self.ui.textEdityMove.setText(str(ret))

            # get z axis
            elif (ofWhatAxis == myBojayIMUAPI.z_axis):
                ret = myBojayIMUAPI.GetDegree_IMUAPIClass(myBojayIMUAPI.z_axis)
                if ret == -9999:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
                self.ui.textEditzMove.setText(str(ret))

        except:
            self.ShowErroeMessage("GetcurrentCoordinate except Fail")
            return -1


    # **************************************************************#
    #get the data from sensor
    def UpdateSensorData(self):
        try:
            ref_readings = []
            average = []
            ret = myBojayIMUAPI.SensorGetDegreeFromSensor_IMUAPIClass(50,0.1,0.05,ref_readings,average)
            if ret != 0:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
            self.ui.labelSensorX.setText(str(average[0]))
            self.ui.labelSensorY.setText(str(average[1]))
            self.ui.labelSensorZ.setText(str(average[2]))
        except:
            self.ShowErroeMessage("UpdateSensorData except Fail")
            return -1


    # **************************************************************#
    # offset PLC
    def OffsetPLC(self,axis):
        try:
            datalist = []
            mean_xyz = []
            #offset x
            if(axis == myBojayIMUAPI.x_axis):
                accuracyx = self.ui.textEditxAccuracyX.toPlainText()
                ret = self.CheckEditBox(True,accuracyx)
                if ret != 0:
                    return -1


                degreex = self.ui.textEditxMove.toPlainText()
                ret = self.CheckEditBox(True,degreex)
                if ret != 0:
                    return -1
                if (float(degreex) <= -180):
                    degreex = 180
                    self.ui.textEditxMove.setText("180")
                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(axis,float(degreex),999999999,float(accuracyx))
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

            # offset y
            elif (axis == myBojayIMUAPI.y_axis):
                accuracyy = self.ui.textEditxAccuracyY.toPlainText()
                ret = self.CheckEditBox(True, accuracyy)
                if ret != 0:
                    return -1

                degreey = self.ui.textEdityMove.toPlainText()
                ret = self.CheckEditBox(True, degreey)
                if ret != 0:
                    return -1
                if (float(degreey) <= -180):
                    degreex = 180
                    self.ui.textEdityMove.setText("180")
                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(axis, float(degreey), 999999999, float(accuracyy))
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

            # offset z
            elif (axis == myBojayIMUAPI.z_axis):
                accuracyz = self.ui.textEditxAccuracyZ.toPlainText()
                ret = self.CheckEditBox(True, accuracyz)
                if ret != 0:
                    return -1

                degreez = self.ui.textEditzMove.toPlainText()
                ret = self.CheckEditBox(True, degreez)
                if ret != 0:
                    return -1
                if (float(degreez) <= -180):
                    degreex = 180
                    self.ui.textEditzMove.setText("180")


                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(axis, float(degreez), 999999999, float(accuracyz))
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

            #self.ShowErroeMessage("OffsetPLC finish")
            return 0
        except:
            self.ShowErroeMessage("OffsetPLC except Fail")
            return -1

    # **************************************************************#
    # create sensor table
    def AcquireSensorDataTable(self):
        try:
            #1:get frequency
            strFrequency = self.ui.textEditFrequency.toPlainText()
            ret = self.CheckEditBox(True, strFrequency)
            if ret != 0:
                    return -1

            #get time
            strTime = self.ui.textEditTime.toPlainText()
            ret = self.CheckEditBox(True, strTime)
            if ret != 0:
                    return -1

            #get sleep time
            strSleepTime = self.ui.textEditSleepTime.toPlainText()
            ret = self.CheckEditBox(True, strSleepTime)
            if ret != 0:
                    return -1

            #get data
            ref_readings = []
            average = []
            ret = myBojayIMUAPI.SensorGetDegreeFromSensor_IMUAPIClass(int(strFrequency),float(strTime),float(strSleepTime),ref_readings,average)
            if ret != 0:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1

            #Save table
            strPath = os.getcwd()
            strPath += "/table.csv"

            myDataTabel = open(strPath,"w")
            myDataTabel.write("Index,Roll,Pitch,Yaw\r")
            for i in range(0,len(ref_readings),1):
                myDataTabel.write(str(i+1) + "," + str(ref_readings[i][0]) + "," + str(ref_readings[i][1]) + "," + str(ref_readings[i][2]) + "\r")
            myDataTabel.close()


            self.ui.labelRollAverage.setText(str(average[0]))
            self.ui.labelPitchAverage.setText(str(average[1]))
            self.ui.labelYawAverage.setText(str(average[2]))
        except:
            self.ShowErroeMessage("AcquireSensorDataTable except Fail")
            return -1


    # **************************************************************#
    # step move
    def StepMove(self,ofWhataxis):
        try:
            if ofWhataxis == myBojayIMUAPI.x_axis:
                xStep = self.ui.textEditXStep.toPlainText()
                ret = self.CheckEditBox(True, xStep)
                if ret != 0:
                    return -1
                ret = myBojayIMUAPI.StepMove_IMUAPIClass(myBojayIMUAPI.x_axis,float(xStep),10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
                #clear
                ret = myBojayIMUAPI.WaitForStepMoveFinishing_IMUAPIClass(ofWhataxis,10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
            elif ofWhataxis == myBojayIMUAPI.y_axis:
                yStep = self.ui.textEditYStep.toPlainText()
                ret = self.CheckEditBox(True, yStep)
                if ret != 0:
                    return -1
                ret = myBojayIMUAPI.StepMove_IMUAPIClass(myBojayIMUAPI.y_axis,float(yStep),10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
                #clear
                ret = myBojayIMUAPI.WaitForStepMoveFinishing_IMUAPIClass(ofWhataxis,10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
            elif ofWhataxis == myBojayIMUAPI.z_axis:
                zStep = self.ui.textEditZStep.toPlainText()
                ret = self.CheckEditBox(True, zStep)
                if ret != 0:
                    return -1
                ret = myBojayIMUAPI.StepMove_IMUAPIClass(myBojayIMUAPI.z_axis,float(zStep),10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
                #clear
                ret = myBojayIMUAPI.WaitForStepMoveFinishing_IMUAPIClass(ofWhataxis,10)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1
            return 0
        except:
            self.ShowErroeMessage("StepMove except Fail")
            return -1

    # **************************************************************#
    # Burning
    def Burning(self):
        try:
            #get loop times
            loopTimes = self.ui.textEditBurningLoopTimes.toPlainText()
            ret = self.CheckEditBox(True, loopTimes)
            if ret != 0:
                return -1
            # get sleep time
            sleepTime = self.ui.textEditBurningInterval.toPlainText()
            ret = self.CheckEditBox(True, sleepTime)
            if ret != 0:
                return -1

            #reset first
            ret = myBojayIMUAPI.SignalReset_IMUAPIClass(20)
            if ret != 0:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1

            # 1:move x axis to 0
            ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(myBojayIMUAPI.x_axis, 0, 10, 5)
            if ret != 0:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1

            #burn start
            for i in  range(0,int(loopTimes),1):

                startTime = time.time()
                #1:move x axis to 90
                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(myBojayIMUAPI.x_axis,90,10,5)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

                #2:move x axis to -90
                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(myBojayIMUAPI.x_axis,-90,10,5)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1


                #3:move x axis to 0
                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(myBojayIMUAPI.x_axis,0,10,5)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1


                #4:move y axis to 90
                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(myBojayIMUAPI.y_axis,90,10,5)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

                #5:move y axis to -90
                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(myBojayIMUAPI.y_axis,-90,10,5)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

                # 6:move y axis to 0
                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(myBojayIMUAPI.y_axis,0,10,5)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

                #8:move x axis to 180
                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(myBojayIMUAPI.x_axis,180,10,5)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

                #7:move x axis to 0
                ret = myBojayIMUAPI.SensorOffsetPLC_IMUAPIClass(myBojayIMUAPI.x_axis,0,10,5)
                if ret != 0:
                    self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                    return -1

                endTime = time.time()
                print(endTime-startTime)
        except:
            self.ShowErroeMessage("Burning except Fail")
            return -1

    # **************************************************************#
    # Set PLC offset
    def SetPLCOffset(self):
        try:
            xOffset = self.ui.textEditXPLCOffset.toPlainText()
            result = self.CheckEditBox(True,xOffset)
            if(result != 0):
                return -1
            yOffset = self.ui.textEditYPLCOffset.toPlainText()
            result = self.CheckEditBox(True,yOffset)
            if(result != 0):
                return -1
            zOffset = self.ui.textEditZPLCOffset.toPlainText()
            result = self.CheckEditBox(True,zOffset)
            if(result != 0):
                return -1



            #set x offset
            ret = myBojayIMUAPI.SetPLCOffset_IMUAPIClass(myBojayIMUAPI.x_axis,float(xOffset))
            if(result != 0):
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1

            #set y offset
            ret = myBojayIMUAPI.SetPLCOffset_IMUAPIClass(myBojayIMUAPI.y_axis,float(yOffset))
            if(result != 0):
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1

            #set z offset
            ret = myBojayIMUAPI.SetPLCOffset_IMUAPIClass(myBojayIMUAPI.z_axis,float(zOffset))
            if(result != 0):
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1

            #save offset
            strPath = os.getcwd() + "/PLCOffset.txt"
            SaveOffset = open(strPath,"w")
            SaveOffset.write("xOffset=" + xOffset + "\r")
            SaveOffset.write("yOffset=" + yOffset + "\r")
            SaveOffset.write("zOffset=" + zOffset + "\r")
            SaveOffset.close()
        except:
            self.ShowErroeMessage("SetPLCOffset except Fail")
            return -1



    # **************************************************************#
    #Check edit box
    def CheckEditBox(self,digital,strValue):
        try:
            if(len(strValue) < 1):
                self.ShowErroeMessage("CheckEditBox:you did not input the value")
                return -1
            # if digital == True:
            #     for i in range(0,len(strValue),1):
            #         if (((strValue[i] != '.') and (strValue[i] != '-'))and (strValue[i] < '0' or strValue[i] > '9')):
            #             self.ShowErroeMessage("CheckEditBox: this edit box only support digital")
            #             return -1
            return 0
        except:
            self.ShowErroeMessage("CheckEditBox except Fail")
            return -1

    # **************************************************************#
    def AcquireSensorAccelerationAndAnglesRate(self):
        try:
            #get data
            acceleration = []
            anglesRate = []
            ret = myBojayIMUAPI.SensorGetAccelerationAndAnglesRate_IMUAPIClass(acceleration, anglesRate)
            if ret != 0:
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1

            self.ui.labelXAcceleration.setText(str(acceleration[0]))
            self.ui.labelYAcceleration.setText(str(acceleration[1]))
            self.ui.labelZAcceleration.setText(str(acceleration[2]))
            self.ui.labelXAnglesRate.setText(str(anglesRate[0]))
            self.ui.labelYAnglesRate.setText(str(anglesRate[1]))
            self.ui.labelZAnglesRate.setText(str(anglesRate[2]))
        except:
            self.ShowErroeMessage("AcquireSensorDataTable except Fail")
            return -1
    # **************************************************************#
    def SetPositionfunction(self):
        try:
            passwd = self.ui.textEditSetPosition.toPlainText()
            if(passwd!="bojay"):
                self.ShowErroeMessage("please input correct passwd:")
                return -1
            ret = myBojayIMUAPI.SetOriginalPosition()
            if (ret!=0):
                self.ShowErroeMessage(myBojayIMUAPI.strErrorMessage)
                return -1
        except:
            self.ShowErroeMessage("SetPositionfunction except Fail")
            return -1

    # **************************************************************#
    # SetLimit
    def SetAxisLimit(self):
        try:
            passwd = self.ui.textEditSetAxisLimit.toPlainText()
            if (passwd != "bojay"):
                self.ShowErroeMessage("please input correct passwd:")
                return -1
            xMaxLimit = self.ui.textEditSetXMaxLimit.toPlainText()
            if(xMaxLimit != ""):
                ret = myBojayIMUAPI.SetLimit_IMUAPIClass(myBojayIMUAPI.max_limit,myBojayIMUAPI.x_axis,xMaxLimit)
                if(ret!=0):
                    self.ShowErroeMessage("fail SetXMaxLimit!")
                    return -1
            xMinLimit = self.ui.textEditSetXMinLimit.toPlainText()
            if(xMinLimit!=""):
                ret = myBojayIMUAPI.SetLimit_IMUAPIClass(myBojayIMUAPI.min_limit,myBojayIMUAPI.x_axis,xMinLimit)
                if(ret!=0):
                    self.ShowErroeMessage("fail SetXMinLimit!")
                    return -1

            yMaxLimit = self.ui.textEditSetYMaxLimit.toPlainText()
            if (yMaxLimit != ""):
                ret = myBojayIMUAPI.SetLimit_IMUAPIClass(myBojayIMUAPI.max_limit, myBojayIMUAPI.y_axis, yMaxLimit)
                if (ret != 0):
                    self.ShowErroeMessage("fail SetYMaxLimit!")
                    return -1
            yMinLimit = self.ui.textEditSetYMinLimit.toPlainText()
            if (yMinLimit != ""):
                ret = myBojayIMUAPI.SetLimit_IMUAPIClass(myBojayIMUAPI.min_limit, myBojayIMUAPI.y_axis, yMinLimit)
                if (ret != 0):
                    self.ShowErroeMessage("fail SetYMinLimit!")
                    return -1

            zMaxLimit = self.ui.textEditSetZMaxLimit.toPlainText()
            if (zMaxLimit != ""):
                ret = myBojayIMUAPI.SetLimit_IMUAPIClass(myBojayIMUAPI.max_limit, myBojayIMUAPI.z_axis, zMaxLimit)
                if (ret != 0):
                    self.ShowErroeMessage("fail SetYMaxLimit!")
                    return -1
            zMinLimit = self.ui.textEditSetZMinLimit.toPlainText()
            if (zMinLimit != ""):
                ret = myBojayIMUAPI.SetLimit_IMUAPIClass(myBojayIMUAPI.min_limit, myBojayIMUAPI.z_axis, zMinLimit)
                if (ret != 0):
                    self.ShowErroeMessage("fail SetYMinLimit!")
                    return -1


        except:
            self.ShowErroeMessage("SetAxisLimit except Fail")
            return -1
            # **************************************************************#
app = QtWidgets.QApplication(sys.argv)
myTonyFrame = TonyFrame()
myTonyFrame.exec_()
exit(app.exec_())