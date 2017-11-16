import termios
import fcntl
import time
import struct
import os
import tty
import select
ZO_PROTOCOL_SET_COMMANDS_START = 0x00
ZO_PROTOCOL_GET_COMMANDS_START = 0x64
ZO_PROTOCOL_BRC_COMMANDS_START = 0xC8
ZO_PROTOCOL_ERROR_ID = 0xFA

ZO_PROTOCOL_HEADER_0 = 0x55
ZO_PROTOCOL_HEADER_1 = 0xAA

ZO_PROTOCOL_BROADCAST_ID = 0x00

ZO_PROTOCOL_DATA_SIZE = 0x16
ZO_PROTOCOL_INFO_SIZE = 0x05
ZO_PROTOCOL_PACKET_SIZE = ZO_PROTOCOL_DATA_SIZE + ZO_PROTOCOL_INFO_SIZE

ZO_PROTOCOL_DO_NOTHING = 0x00
ZO_PROTOCOL_STOP = 0x01
ZO_PROTOCOL_HALT = 0x02
ZO_PROTOCOL_COMMAND_RESPONSE_TIMEOUT_US = 15000
ZO_WARNING_NONE = 0
ZO_WARNING_WRONG_LRC = 104
ZO_WARNING_RESPONSE_TIMEOUT = 105
ZO_WARNING_SERIAL_PORT = 106
WAIT_ON_HEADER_0 = 0
WAIT_ON_HEADER_1 = 1
WAIT_ON_ADDRESSED_NODE_ID = 2
WAIT_ON_OWN_NODE_ID = 3
WAIT_ON_COMMAND_ID = 4
WAIT_ON_BYTECOUNT = 5
WAIT_ON_DATA = 6
WAIT_ON_LRC = 7
ZO_PROTOCOL_DECODER_STATE = {
    "WAIT_ON_HEADER_0": WAIT_ON_HEADER_0,
    "WAIT_ON_HEADER_1": WAIT_ON_HEADER_1,
    "WAIT_ON_ADDRESSED_NODE_ID": WAIT_ON_ADDRESSED_NODE_ID,
    "WAIT_ON_OWN_NODE_ID": WAIT_ON_OWN_NODE_ID,
    "WAIT_ON_COMMAND_ID": WAIT_ON_COMMAND_ID,
    "WAIT_ON_BYTECOUNT": WAIT_ON_BYTECOUNT,
    "WAIT_ON_DATA": WAIT_ON_DATA,
    "WAIT_ON_LRC": WAIT_ON_LRC}
decoderState = WAIT_ON_HEADER_0
# global CommSuccess, warning
CommSuccess = True
warning = ZO_WARNING_NONE


class zo_protocol_packet(object):
    def __init__(self, addressedNodeID, ownNodeID, commandID, byteCount, data):
        self.addressedNodeID = addressedNodeID
        self.ownNodeID = ownNodeID
        self.commandID = commandID
        self.byteCount = byteCount
        self.lrc = self.calcLRC()
        self.data = data

    def getResponse(self, serial_object):
        # // tcflush(fd, TCIFLUSH);
        global warning, CommSuccess
        while(not(self.getPacketSerial(serial_object))):
            if not serial_object.serialTimeout(ZO_PROTOCOL_COMMAND_RESPONSE_TIMEOUT_US):
                # // printf("DEBUG: timeout\n");
                CommSuccess = False
                warning = ZO_WARNING_RESPONSE_TIMEOUT
                # print(CommSuccess, "ZO_WARNING_RESPONSE_TIMEOUT")
                break
        if(CommSuccess is True):
            if(self.lrc != self.calcLRC()):
                CommSuccess = False
                warning = ZO_WARNING_WRONG_LRC
                print(CommSuccess, "ZO_WARNING_WRONG_LRC")
            if(self.commandID == ZO_PROTOCOL_ERROR_ID):
                CommSuccess = False
                warning = self.data[0]
        return CommSuccess

    def calcLRC(self):
        lrc = 0
        lrc ^= self.commandID
        lrc ^= self.byteCount
        for i in range(0, self.byteCount):
            if type(self.data[i]) is str:
                # print(self.data[i])
                local_data = struct.unpack('<B', self.data[i])[0]
            else:
                local_data = self.data[i]
            lrc ^= local_data
        return lrc

    def putPacketSerial(self, serial_object):
        # /* Also we can check the return value of writeByte function */
        serial_object.writeByte(ZO_PROTOCOL_HEADER_0)
        serial_object.writeByte(ZO_PROTOCOL_HEADER_1)
        serial_object.writeByte(self.addressedNodeID)
        serial_object.writeByte(self.ownNodeID)
        serial_object.writeByte(self.commandID)
        serial_object.writeByte(self.byteCount)
        for i in range(0, self.byteCount):
            serial_object.writeByte(self.data[i])
        serial_object.writeByte(self.lrc)
        return True

    def getPacketSerial(self, serial_object):
        global decoderState, byteCount
        isWholePacket = False
        c = os.read(serial_object.fd, 1)

        if (c != 1):
            isWholePacket = False
        if (c == ''):
            isWholePacket = False
            return False
        # // printf("DEBUG: %x, %s (%d)\n", c, strerror(errno), errno);
        if decoderState == WAIT_ON_HEADER_0:
            if (struct.unpack('<B', c)[0] == ZO_PROTOCOL_HEADER_0):
                decoderState = WAIT_ON_HEADER_1
            else:
                decoderState = WAIT_ON_HEADER_0
        elif decoderState == WAIT_ON_HEADER_1:
            if(struct.unpack('<B', c)[0] == ZO_PROTOCOL_HEADER_1):
                decoderState = WAIT_ON_ADDRESSED_NODE_ID
            else:
                decoderState = WAIT_ON_HEADER_0
        elif decoderState == WAIT_ON_ADDRESSED_NODE_ID:
            if(struct.unpack('<B', c)[0] == 0x01):
                decoderState = WAIT_ON_OWN_NODE_ID
                self.addressedNodeID = struct.unpack('<B', c)[0]
            else:
                decoderState = WAIT_ON_HEADER_0
        elif decoderState == WAIT_ON_OWN_NODE_ID:
            self.ownNodeID = struct.unpack('<B', c)[0]
            decoderState = WAIT_ON_COMMAND_ID
        elif decoderState == WAIT_ON_COMMAND_ID:
            self.commandID = struct.unpack('<B', c)[0]
            decoderState = WAIT_ON_BYTECOUNT
        elif decoderState == WAIT_ON_BYTECOUNT:
            self.byteCount = struct.unpack('<B', c)[0]
            byteCount = self.byteCount
            if(byteCount > 0):
                decoderState = WAIT_ON_DATA
            else:
                decoderState = WAIT_ON_LRC
        elif decoderState == WAIT_ON_DATA:
            # print(byteCount)
            # print(self.byteCount)
            self.data.append(c)
            byteCount -= 1
            if(byteCount == 0):
                decoderState = WAIT_ON_LRC
        elif decoderState == WAIT_ON_LRC:
            self.lrc = struct.unpack('<B', c)[0]
            decoderState = WAIT_ON_HEADER_0
            isWholePacket = True
        # print(isWholePacket)
        return isWholePacket


class zo_sms_serial(object):

    def serialPortOpen(self, serialport, baud_rate):
        # options = termios()
        # int status, fd;
        global warning, CommSuccess
        fd = os.open(serialport, os.O_RDWR | os.O_NOCTTY | os.O_NDELAY | os.O_NONBLOCK)
        if (fd == -1):
            raise OSError("serialport_init: Unable to open port")
            CommSuccess = False
            warning = ZO_WARNING_SERIAL_PORT
            return -1
        # check this page for more help: https://docs.python.org/2/library/termios.html#module-termios
        # and http://epydoc.sourceforge.net/stdlib/termios-module.html
        fcntl.fcntl(fd, fcntl.F_SETFL, os.O_RDWR)
        options = termios.tcgetattr(fd)
        tty.setraw(fd)
        options[4] = 4097
        options[5] = 4097
        # /* 8N1 */
        options[2] |= (termios.CLOCAL | termios.CREAD)
        options[2] &= ~termios.PARENB
        options[2] &= ~termios.CSTOPB
        options[2] &= ~termios.CSIZE
        options[2] |= termios.CS8
        options[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
        # check also this page for help: http://man7.org/linux/man-pages/man3/termios.3.html
        options[6][9] = chr(0)
        options[6][17] = chr(0)
        termios.tcsetattr(fd, termios.TCSANOW | termios.TCSAFLUSH, options)
        status = struct.pack('I', 0)
        fcntl.ioctl(fd, tty.TIOCMGET, status)
        status = struct.unpack('I', status)[0]
        status |= termios.TIOCM_DTR
        status |= termios.TIOCM_RTS
        status = struct.pack('I', status)
        fcntl.ioctl(fd, termios.TIOCMSET, status)
        time.sleep(0.01)
        try:
            termios.tcsetattr(fd, termios.TCSAFLUSH, options)
        except OSError:
            raise OSError("init_serialport: Couldn't set term attributes")
            return -1
        self.fd = fd
        print(fd)
        # return fd

    def serialPortClose(self):
        return os.close(self.fd)

    def writeByte(self, b):
        # print(b)
        if type(b) is str:
            n = os.write(self.fd, b)
        else:
            n = os.write(self.fd, struct.pack('<B', b))
        if(n != 1):
            return -1
        return 0

    def serialTimeout(self, usec):
        timeout = float(usec) / (10**6)
        # print(timeout)
        # /* select returns 0 if timeout, 1 if input available, -1 if error. */
        # print(self.fd)
        return select.select([self.fd], [], [], timeout)[0]
        # return select.select([self.fd], [], [], timeout)
        # return 0

    def broadcastDoMove(self):
        packet = zo_protocol_packet.create_packet(0x00, 0x01, 0xC8, 0x00, 0xC8, [])
        return packet.putPacketSerial()

    def broadcastStart(self):
        packet = zo_protocol_packet.create_packet(0x00, 0x01, 0xC9, 0x00, 0xC9, [])
        return packet.putPacketSerial()
        # command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xC9, 0x00, 0xC9])
        # ser.write(command)
        # return True

    def broadcastHalt(self):
        packet = zo_protocol_packet.create_packet(0x00, 0x01, 0xCA, 0x00, 0xCA, [])
        return packet.putPacketSerial()
        # command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xCA, 0x00, 0xCA])
        # ser.write(command)
        # return True

    def broadcastStop(self):
        packet = zo_protocol_packet.create_packet(0x00, 0x01, 0xCB, 0x00, 0xCB, [])
        return packet.putPacketSerial()
        # command = bytearray([0x55, 0xAA, 0x00, 0x01, 0xCB, 0x00, 0xCB])
        # ser.write(command)
        # return True

    def getCommunicationSuccess(self):
        global CommSuccess
        success = CommSuccess  # /*store to local*/
        CommSuccess = True     # /*initialize for next comm*/
        return success

    def getWarning(self):
        global warning
        warn = warning  # /*store to local*/
        warning = ZO_WARNING_NONE  # /*clear warning*/
        return warn


class sms_motor(object):

    def __init__(self, serial, motorId, resolution_bits):
        self.motorId = motorId
        self.resolution_bits = resolution_bits
        self.serial = serial
        # self.limits = limits
        # self.homing_position_ticks = homing_position_ticks

    def setPIDgainP(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x00, 0x02, struct.pack('<h', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
            # self.send2Bytes(0, val)
            # return not self.getResponse()[0]

    def setPIDgainI(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x01, 0x02, struct.pack('<h', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)

    def setPIDgainD(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x02, 0x02, struct.pack('<h', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send2Bytes(2, val)
        # return not self.getResponse()[0]

    def setProfileAcceleration(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x03, 0x04, struct.pack('<f', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send4Bytes(3, val)
        # return not self.getResponse()[0]

    def setProfileConstantVelocity(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x04, 0x04, struct.pack('<f', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send4Bytes(4, val)
        # return not self.getResponse()[0]

    def setCurrentLimit(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x05, 0x02, struct.pack('<h', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send2Bytes(5, val)
        # return not self.getResponse()[0]

    def setDurationForCurrentLimit(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x06, 0x02, struct.pack('<h', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send2Bytes(6, val)
        # return not self.getResponse()[0]

    def moveWithVelocity(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x07, 0x04, struct.pack('<f', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send4Bytes(7, val)
        # return not self.getResponse()[0]

    def moveToAbsolutePosition(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x08, 0x08, struct.pack('<d', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send8Bytes(8, pos)
        # return not self.getResponse()[0]

    def moveToRelativePosition(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x09, 0x08, struct.pack('<d', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send8Bytes(9, pos)
        # return not self.getResponse()[0]

    def profiledMoveWithVelocity(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x0A, 0x04, struct.pack('<f', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send4Bytes(10, val)
        # return not self.getResponse()[0]

    def profiledMoveToAbsolutePosition(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x0B, 0x08, struct.pack('<d', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send8Bytes(11, pos)
        # return not self.getResponse()[0]

    def profiledMoveToRelativePosition(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x0C, 0x08, struct.pack('<d', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send8Bytes(12, pos)
        # return not self.getResponse()[0]

    def setVelocitySetpoint(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x0D, 0x04, struct.pack('<f', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send4Bytes(13, val)
        # return not self.getResponse()[0]

    def setAbsolutePositionSetpoint(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x0E, 0x08, struct.pack('<d', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send8Bytes(14, pos)
        # return not self.getResponse()[0]

    def setRelativePositionSetpoint(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x0F, 0x08, struct.pack('<d', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send8Bytes(15, pos)
        # return not self.getResponse()[0]

    def setProfiledVelocitySetpoint(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x10, 0x04, struct.pack('<f', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send4Bytes(16, val)
        # return not self.getResponse()[0]

    def setProfiledAbsolutePositionSetpoint(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x11, 0x08, struct.pack('<d', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send8Bytes(17, pos)
        # return not self.getResponse()[0]

    def setProfiledRelativePositionSetpoint(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x12, 0x08, struct.pack('<d', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.send8Bytes(18, pos)
        # return not self.getResponse()[0]

    def configureDigitalIOs(self, dio1, dio2, dio3):
        data = {}
        data[0] = 0
        if (dio1):
            data[0] |= 0x01
        if (dio2):
            data[0] |= 0x02
        if (dio3):
            data[0] |= 0x04
        packet = zo_protocol_packet(self.motorId, 0x01, 0x13, 0x01, struct.pack('<B', data))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.sendByte(19, data[0])
        # return not self.getResponse()[0]

    def setDigitalOutputs(self, dio1, dio2, dio3):
        data = {}
        data[0] = 0
        if (dio1):
            data[0] |= 0x01
        if (dio2):
            data[0] |= 0x02
        if (dio3):
            data[0] |= 0x04
        packet = zo_protocol_packet(self.motorId, 0x01, 0x14, 0x01, struct.pack('<B', data))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.sendByte(20, data[0])
        # return not self.getResponse()[0]

    def setNodeID(self, oldNodeId, newNodeId):
        data = {}
        data[0] = newNodeId
        packet = zo_protocol_packet(self.motorId, 0x01, 0x15, 0x01, struct.pack('<B', data))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # lrc = self.checksum(21, 1, data)
        # command = bytearray(['\x55', '\xAA', oldNodeId, '\x01', '\x15', '\x01', data[0], lrc])
        # ser.write(command)
        # return not self.getResponse()[0]

    def setBaudRate(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x17, 0x04, struct.pack('<f', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)

    def resetIncrementalPosition(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x18, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.sendCommand(24)
        # return not self.getResponse()[0]

    def start(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x19, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.sendCommand(25)
        # # time.sleep(0.02)
        # return not self.getResponse()[0]

    def halt(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x1A, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.sendCommand(26)
        # # time.sleep(0.02)
        # return not self.getResponse()[0]

    def stop(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x1B, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)
        # self.sendCommand(27)
        # # time.sleep(0.02)
        # return not self.getResponse()[0]

    def setErrorReaction(self, resp):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x1C, 0x14, resp)
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)

    def setAntiWindup(self, val):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x1D, 0x04, struct.pack('<f', val))
        if(packet.putPacketSerial(self.serial)):
            return packet.getResponse(self.serial)

    def resetErrors(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x1E, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            print("Packet Sent")
            return packet.getResponse(self.serial)
        # self.sendCommand(30)
        # # time.sleep(0.02)
        # return not self.getResponse()[0]

    def getPIDgainP(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x64, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<h', packet.data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(100)
        # return self.getResponse()

    def getPIDgainI(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x65, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<h', packet.data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(101)
        # return self.getResponse()

    def getPIDgainD(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x66, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<h', packet.data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(102)
        # return self.getResponse()

    def getProfileAcceleration(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x67, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<f', packet.data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(103)
        # return self.getResponse()

    def getProfileConstantVelocity(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x68, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<f', packet.data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(104)
        # return self.getResponse()

    def getCurrentLimit(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x69, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<h', packet.data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(105)
        # return self.getResponse()

    def getCurrentLimitDuration(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x6A, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<h', packet.data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(106)
        # return self.getResponse()

    def getDigitalIOConfiguration(self, dio):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x6B, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return ((packet.data[0] & (1 << dio)) == (1 << dio))
            else:
                return -1
        else:
            return -1
        # self.sendCommand(107)
        # return self.getDigitalIOResponse()

    def getDigitalIn(self, din):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x6D, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return ((packet.data[0] & (1 << din)) == (1 << din))
            else:
                return -1
        else:
            return -1
        # self.sendCommand(109)
        # return self.getDigitalIOResponse()

    def getAnalogIn(self, ain):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x6E, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<h', packet.data[(ain * 2) - 2])[0]
                # probably a bug here!!!
            else:
                return -1
        else:
            return -1

    def getPosition(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x6F, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                data = ''.join(str(elem) for elem in packet.data)
                return struct.unpack('<q', data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(111)
        # return self.getResponse()

    def getAbsolutePosition(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x70, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            # print("Packet Sent")
            if packet.getResponse(self.serial):
                # print("Got Response")
                data = ''.join(str(elem) for elem in packet.data)
                return struct.unpack('<H', data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(112)
        # return self.getResponse()

    def getVelocity(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x71, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<f', packet.data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(113)
        # return self.getResponse()

    def getCurrent(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x72, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<h', packet.data)[0]
            else:
                return -1
        else:
            return -1
        # self.sendCommand(114)
        # return self.getResponse()

    def getErrorReaction(self, resp):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x73, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                resp = packet.data
            else:
                return False
        else:
            return False

    def getAntiWindup(self):
        packet = zo_protocol_packet(self.motorId, 0x01, 0x74, 0x00, [])
        if(packet.putPacketSerial(self.serial)):
            if packet.getResponse(self.serial):
                return struct.unpack('<f', packet.data)[0]
            else:
                return -1
        else:
            return -1
