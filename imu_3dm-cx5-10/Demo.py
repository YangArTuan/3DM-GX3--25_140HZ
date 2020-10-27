# import the mscl library
import platform
try:
    OperatingSystem = platform.system()
    if OperatingSystem == "Windows":
        import mscl_windows as mscl
    elif OperatingSystem == "Linux":
        import mscl_linux as mscl
except Exception as ex:
    print(ex)

communicateType = True # False:pullData  True:continual communicate

# # ping test
# ping = node.ping()
# print("Ping :",ping)

def pullDataCommunicate(node):
    try:
        # pull data command
        node.pollData(mscl.MipTypes.CLASS_AHRS_IMU)

        # get all the data packets from the node, with a timeout of 500 milliseconds
        packets = node.getDataPackets(timeout=500, maxPackets=1)

        for packet in packets:

            # print out the data
            print("Packet Received: ")

            # iterate over all the data points in the packet
            for dataPoint in packet.data():
                # print out the channel data
                # Note: The as_string() function is being used here for simplicity.
                #      Other methods (ie. as_float, as_uint16, as_Vector) are also available.
                #      To determine the format that a dataPoint is stored in, use dataPoint.storedAs().
                print(dataPoint.channelName() + ":", dataPoint.as_string() + " ")

                # if the dataPoint is invalid
                if (dataPoint.valid() == False):
                    print("[Invalid] ")
    except Exception as ex:
        print(ex)


def ContinualCommunicate(node):
    try:
        '''
        # if the node supports AHRS/IMU
        if node.features().supportsCategory(mscl.MipTypes.CLASS_AHRS_IMU):
            node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU, enable=False)

        # if the node supports Estimation Filter
        if node.features().supportsCategory(mscl.MipTypes.CLASS_ESTFILTER):
            node.enableDataStream(mscl.MipTypes.CLASS_ESTFILTER, enable=False)

        # if the node supports GNSS
        if node.features().supportsCategory(mscl.MipTypes.CLASS_GNSS):
            node.enableDataStream(mscl.MipTypes.CLASS_GNSS, enable=False)
        '''

        # get all the data packets from the node, with a timeout of 500 milliseconds
        packets = node.getDataPackets(timeout=500, maxPackets=1)

        for packet in packets:

            # print out the data
            print("Packet Received: ")

            # iterate over all the data points in the packet
            for dataPoint in packet.data():
                # print out the channel data
                # Note: The as_string() function is being used here for simplicity.
                #      Other methods (ie. as_float, as_uint16, as_Vector) are also available.
                #      To determine the format that a dataPoint is stored in, use dataPoint.storedAs().
                print(dataPoint.channelName() + ":", dataPoint.as_string() + " ")

                # if the dataPoint is invalid
                if (dataPoint.valid() == False):
                    print("[Invalid] ")
    except Exception as ex:
        print(ex)



if __name__ == '__main__':
    # TODO: change these constants to match your setup
    COM_PORT = "COM24"

    # create a Serial Connection with the specified COM Port, default baud rate of 921600
    connection = mscl.Connection.Serial(COM_PORT, 115200)

    # create an InertialNode with the connection
    node = mscl.InertialNode(connection)

    '''
    insert your process code
    '''
    if (communicateType == True):
        # if the node supports AHRS/IMU
        if node.features().supportsCategory(mscl.MipTypes.CLASS_AHRS_IMU):
            node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU)

        # if the node supports Estimation Filter
        if node.features().supportsCategory(mscl.MipTypes.CLASS_ESTFILTER):
            node.enableDataStream(mscl.MipTypes.CLASS_ESTFILTER)

        # if the node supports GNSS
        if node.features().supportsCategory(mscl.MipTypes.CLASS_GNSS):
            node.enableDataStream(mscl.MipTypes.CLASS_GNSS)
    elif (communicateType == False):
        # set to idle
        node.setToIdle()

        # save setting
        node.saveSettingsAsStartup()

    while (True):
        if (communicateType == True):
            ContinualCommunicate(node=node)
        elif (communicateType == False):
            pullDataCommunicate(node=node)

    # # close serial port
    # disconnection = connection.disconnect()