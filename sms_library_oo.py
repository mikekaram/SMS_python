import serial
# import time
# import sys
import platform


class sms_motor(object):

    def __init__(self, motorId, resolution_bits, limits, homing_position_ticks):
        self.motorId = motorId
        self.resolution_bits = resolution_bits
        self.limits = limits
        self.homing_position_ticks = homing_position_ticks

    @staticmethod
    def checksum(cmd, bc, x):
        ret = 0
        ret ^= (cmd ^ bc)
        for i in range(0, bc):
            ret ^= x[i]
        return ret

    @staticmethod
    def getAnalogInResponse():
        success = False
        b = ser.read(6)
        # The first 6 bytes are useless for us (2 header bytes + addressed and owned id + command id and byte count = 6 bytes)
        ain = [0, 0, 0, 0]
        if ord(b[5]) != 4:
            return success, ain
        success = True
        for i in range(0, 4):
            shift = 0
            for j in range(0, 2):
                b += ser.read()
                ain[i] += (ord(b[2 * i + j + 6]) & 0xFF) << shift
                shift += 8
        b += ser.read()
        # necessary for not leaving any garbage in the serial read interface (we have to read the lrc byte)
        return success, ain

    @staticmethod
    def getDigitalIOResponse():
        success = False
        b = ser.read(6)
        dio1 = 0
        dio2 = 0
        dio3 = 0
        # The first 6 bytes are useless for us (2 header bytes + addressed and owned id + command id and byte count = 6 bytes)
        if ord(b[5]) != 1:
            return success, dio1, dio2, dio3
        else:
            success = True
            b += ser.read()
            dio1 = ord(b[6]) & 0x01
            dio2 = ord(b[6]) & 0x02
            dio3 = ord(b[6]) & 0x04
        b += ser.read()
        # necessary for not leaving any garbage in the serial read interface (we have to read the lrc byte)
        return success, dio1, dio2, dio3

    @staticmethod
    def getResponse():
        carrying_data = False
        b = ser.read(6)
        # The first 6 bytes are useless for us (2 header bytes + addressed and owned id + command id and byte count = 6 bytes)
        num = 0
        print("Read 6 bytes and byte count is:", ord(b[5]))
        for i in range(0, ord(b[5])):
            carrying_data = True
            b += ser.read()
            shift = i * 8
            num += (ord(b[i + 6]) & 0xFF) << shift
        b += ser.read()
        # necessary for not leaving any garbage in the serial read interface (we have to read the lrc byte)
        return carrying_data, num

    # @staticmethod
    # def print_message():
    #     print("Usage: [sudo] python SMSLibrary.py port_number command [Node Id] [value]\n")
    #     print("Commands are: start [Node Id], stop [Node Id], reset [Node Id], startall, stopall, resetall,\n")
    #     print("getvel [Node Id], getacc [Node Id], getpos [Node Id],\n")
    #     print("move [Node Id] [start] [goal], setvel [Node Id] [value], setacc [Node Id] [value], test [Node Id]")

    def send8Bytes(self, cmd_id, value):
        data = {}
        data[7] = (value >> 56) & 0xff
        data[6] = (value >> 48) & 0xff
        data[5] = (value >> 40) & 0xff
        data[4] = (value >> 32) & 0xff
        data[3] = (value >> 24) & 0xff
        data[2] = (value >> 16) & 0xff
        data[1] = (value >> 8) & 0xff
        data[0] = value & 0xff
        lrc = self.checksum(cmd_id, 8, data)
        command = bytearray(
            [h_0, h_1, self.motorId, '\x01', chr(cmd_id), '\x08', data[0], data[1], data[2], data[3], data[4], data[5], data[6],
             data[7], lrc])
        ser.write(command)

    def send4Bytes(self, cmd_id, value):
        data = {}
        data[3] = (value >> 24) & 0xff
        data[2] = (value >> 16) & 0xff
        data[1] = (value >> 8) & 0xff
        data[0] = value & 0xff
        lrc = self.checksum(cmd_id, 4, data)
        command = bytearray(
            [h_0, h_1, self.motorId, '\x01', chr(cmd_id), '\x04', data[0], data[1], data[2], data[3], lrc])
        ser.write(command)

    def send2Bytes(self, cmd_id, value):
        data = {}
        data[1] = (value >> 8) & 0xff
        data[0] = value & 0xff
        lrc = self.checksum(cmd_id, 2, data)
        command = bytearray([h_0, h_1, self.motorId, '\x01', chr(cmd_id), '\x02', data[0], data[1], lrc])
        ser.write(command)

    def sendByte(self, cmd_id, value):
        lrc = self.checksum(cmd_id, 1, value)
        command = bytearray([h_0, h_1, self.motorId, '\x01', chr(cmd_id), '\x01', value, lrc])
        ser.write(command)

    def sendCommand(self, cmd_id):
        command = bytearray([h_0, h_1, self.motorId, '\x01', chr(cmd_id), '\x00', chr(cmd_id)])
        ser.write(command)

    ###############################################################################################################################
    # Important! In Set Commands we return the not getResponse()[0] because getResponse() returns as it's first argument a boolean
    # variable, carrying_data, which represents if the response carries any data. Because the response of the Set Commands
    # doesn't carry any data, the carrying_data will be False (but the response to the command was successful).
    #  So we negate the getResponse()[0] to give as True for these commands.
    ###############################################################################################################################

    def setPIDgainP(self, val):
        self.send2Bytes(0, val)
        return not self.getResponse()[0]

    def setPIDgainI(self, val):
        self.send2Bytes(1, val)
        return not self.getResponse()[0]

    def setPIDgainD(self, val):
        self.send2Bytes(2, val)
        return not self.getResponse()[0]

    def setProfileAcceleration(self, val):
        self.send4Bytes(3, val)
        return not self.getResponse()[0]

    def setProfileConstantVelocity(self, val):
        self.send4Bytes(4, val)
        return not self.getResponse()[0]

    def setCurrentLimit(self, val):
        self.send2Bytes(5, val)
        return not self.getResponse()[0]

    def setDurationForCurrentLimit(self, val):
        self.send2Bytes(6, val)
        return not self.getResponse()[0]

    def moveWithVelocity(self, val):
        self.send4Bytes(7, val)
        return not self.getResponse()[0]

    def moveToAbsolutePosition(self, pos):
        self.send8Bytes(8, pos)
        return not self.getResponse()[0]

    def moveToRelativePosition(self, pos):
        self.send8Bytes(9, pos)
        return not self.getResponse()[0]

    def profiledMoveWithVelocity(self, val):
        self.send4Bytes(10, val)
        return not self.getResponse()[0]

    def profiledMoveToAbsolutePosition(self, pos):
        self.send8Bytes(11, pos)
        return not self.getResponse()[0]

    def profiledMoveToRelativePosition(self, pos):
        self.send8Bytes(12, pos)
        return not self.getResponse()[0]

    def setVelocitySetpoint(self, val):
        self.send4Bytes(13, val)
        return not self.getResponse()[0]

    def setAbsolutePositionSetpoint(self, pos):
        self.send8Bytes(14, pos)
        return not self.getResponse()[0]

    def setRelativePositionSetpoint(self, pos):
        self.send8Bytes(15, pos)
        return not self.getResponse()[0]

    def setProfiledVelocitySetpoint(self, val):
        self.send4Bytes(16, val)
        return not self.getResponse()[0]

    def setProfiledAbsolutePositionSetpoint(self, pos):
        self.send8Bytes(17, pos)
        return not self.getResponse()[0]

    def setProfiledRelativePositionSetpoint(self, pos):
        self.send8Bytes(18, pos)
        return not self.getResponse()[0]

    def configureDigitalIOs(self, dio1, dio2, dio3):
        data = {}
        data[0] = 0
        if (dio1):
            data[0] |= 0x01
        if (dio2):
            data[0] |= 0x02
        if (dio3):
            data[0] |= 0x04
        self.sendByte(19, data[0])
        return not self.getResponse()[0]

    def setDigitalOutputs(self, dio1, dio2, dio3):
        data = {}
        data[0] = 0
        if (dio1):
            data[0] |= 0x01
        if (dio2):
            data[0] |= 0x02
        if (dio3):
            data[0] |= 0x04
        self.sendByte(20, data[0])
        return not self.getResponse()[0]

    def setNodeID(self, oldNodeId, newNodeId):
        data = {}
        data[0] = newNodeId
        lrc = self.checksum(21, 1, data)
        command = bytearray(['\x55', '\xAA', oldNodeId, '\x01', '\x15', '\x01', data[0], lrc])
        ser.write(command)
        return not self.getResponse()[0]

    def resetIncrementalPosition(self):
        self.sendCommand(24)
        return not self.getResponse()[0]

    def start(self):
        self.sendCommand(25)
        # time.sleep(0.02)
        return not self.getResponse()[0]

    def halt(self):
        self.sendCommand(26)
        # time.sleep(0.02)
        return not self.getResponse()[0]

    def stop(self):
        self.sendCommand(27)
        # time.sleep(0.02)
        return not self.getResponse()[0]

    def resetErrors(self):
        self.sendCommand(30)
        # time.sleep(0.02)
        return not self.getResponse()[0]

    def getPIDgainP(self):
        self.sendCommand(100)
        return self.getResponse()

    def getPIDgainI(self):
        self.sendCommand(101)
        return self.getResponse()

    def getPIDgainD(self):
        self.sendCommand(102)
        return self.getResponse()

    def getProfileAcceleration(self):
        self.sendCommand(103)
        return self.getResponse()

    def getProfileConstantVelocity(self):
        self.sendCommand(104)
        return self.getResponse()

    def getCurrentLimit(self):
        self.sendCommand(105)
        return self.getResponse()

    def getCurrentLimitDuration(self):
        self.sendCommand(106)
        return self.getResponse()

    def getDigitalIOConfiguration(self, dio):
        self.sendCommand(107)
        return self.getDigitalIOResponse()

    def getDigitalIn(self, din):
        self.sendCommand(109)
        return self.getDigitalIOResponse()

    def getAnalogIn(self, ain):
        self.sendCommand(110)
        return self.getAnalogInResponse()

    def getPosition(self):
        self.sendCommand(111)
        return self.getResponse()

    def getAbsolutePosition(self):
        self.sendCommand(112)
        return self.getResponse()

    def getVelocity(self):
        self.sendCommand(113)
        return self.getResponse()

    def getCurrent(self):
        self.sendCommand(114)
        return self.getResponse()


def broadcastDoMove():
    command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xC8, 0x00, 0xC8])
    ser.write(command)
    return True


def broadcastStart():
    command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xC9, 0x00, 0xC9])
    ser.write(command)
    return True


def broadcastHalt():
    command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xCA, 0x00, 0xCA])
    ser.write(command)
    return True


def broadcastStop():
    command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xCB, 0x00, 0xCB])
    ser.write(command)
    return True


def init(port):
    global ser, h_0, h_1
    h_0 = '\x55'
    h_1 = '\xAA'
    port_string = ''
    if(platform.system() == "Windows"):
        port_string = 'COM%d' % port
    elif(platform.system() == "Linux"):
        port_string = '/dev/ttyUSB%d' % port
    ser = serial.Serial(port_string, 57600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)


def shut_down(port):
    global ser
    if ser.isOpen():
        ser.close()
    exit(0)


# def startall(startId, stopId):
#     for i in range(startId, stopId + 1):
#         start(i)


# def stopall(startId, stopId):
#     for i in range(startId, stopId + 1):
#         stop(i)


# def resetall(startId, stopId):
#     for i in range(startId, stopId + 1):
#         print(resetErrors(i))

# def main():

#     if (len(sys.argv) < 3):
#         print("Too few input arguments.\n")
#         print_message()
#         if(len(sys.argv) < 2):
#             exit(0)
#         else:
#             shut_down(int(sys.argv[1]))
#     elif (len(sys.argv) > 5):
#         print("Too many input arguments.\n")
#         print_message()
#         shut_down(int(sys.argv[1]))

#     init(int(sys.argv[1]))

#     if (sys.argv[2] == "getvel"):
#         ret = getVelocity(int(sys.argv[3]))
#         for b in ret:
#             print(ord(b))
#         shut_down(int(sys.argv[1]))

#     if (sys.argv[2] == "getpos"):
#         mid = int(sys.argv[3])
#         pos = getPosition(mid)
#         print(pos[0], float(pos[1]))
#         shut_down(int(sys.argv[1]))

#     if (sys.argv[2] == "getacc"):
#         ret = getProfileAcceleration(int(sys.argv[3]))
#         for b in ret:
#             print(ord(b))
#         shut_down(int(sys.argv[1]))
#     if (sys.argv[2] == "setvel"):
#         setProfileConstantVelocity(int(sys.argv[3]), int(sys.argv[4]))
#         shut_down(int(sys.argv[1]))
#     if (sys.argv[2] == "setacc"):
#         setProfileAcceleration(int(sys.argv[3]), int(sys.argv[4]))
#         shut_down(int(sys.argv[1]))

#     if (sys.argv[2] == "startall"):
#         broadcastStart()
#         shut_down(int(sys.argv[1]))
#     if (sys.argv[2] == "start"):
#         start(int(sys.argv[3]))
#         shut_down(int(sys.argv[1]))
#     if (sys.argv[2] == "stopall"):
#         broadcastStop()
#         shut_down(int(sys.argv[1]))
#     if (sys.argv[2] == "stop"):
#         stop(int(sys.argv[3]))
#         shut_down(int(sys.argv[1]))
#     if (sys.argv[2] == "resetall"):
#         resetall(4, 6)
#         shut_down(int(sys.argv[1]))
#     if (sys.argv[2] == "reset"):
#         resetErrors(int(sys.argv[3]))
#         shut_down(int(sys.argv[1]))

#     if (sys.argv[2] == "move"):
#         profiledMoveToAbsolutePosition(int(sys.argv[3]), int(sys.argv[4]))
#         time.sleep(0.02)
#         shut_down(int(sys.argv[1]))

#     if (sys.argv[2] == "test"):
#         angle = 0
#         direction = 1
#         step = 10
#         while 1:

#             # move(5, 1722*angle)
#             # time.sleep(0.01)
#             profiledMoveToAbsolutePosition(int(sys.argv[3]), angle)

#             print(angle)
#             angle = angle + direction * step
#             if int(angle) == 0:
#                 direction = 1
#             if int(angle) == 15000:
#                 direction = -1
#             time.sleep(0.03)


# if __name__ == '__main__':
#     main()
