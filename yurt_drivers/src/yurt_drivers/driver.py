import serial, time, sys, thread
from struct import unpack
from constants import *

# @driver.py
class Driver():
    def __init__(self, id, port='/dev/ttyUSB0', baud=115200, timeout=0.1):
        self._mutex = thread.allocate_lock()
        self._id = id
        self._ser = serial.Serial()
        self._ser.baudrate = baud
        self._ser.port = port
        self._ser.timeout = timeout
        self._ser.open()
    
    def setSpeed(self, params, speeds):
        if type(params) is not list: params = [params]
        if type(speeds) is not list: speeds = [speeds]
        
        message = []
        for param,speed in zip(params,speeds):
            if speed < 0:
                speed = abs(speed)
                direction = MOTOR_DIRECTION_CCW
            else:
                direction = MOTOR_DIRECTION_CW
            message.extend([param, (speed >> 8) & 0xFE , (speed & 0xFE) ,param+16,0x00,direction])
        self.execute(INST_WRITE, message)
    
    def getSpeed(self, params):
        if type(params) is not list: params = [params]
        finalParams = []
        for param in params:
            finalParams.append(param)
            finalParams.append(param+16)
        values = self.execute(INST_READ, finalParams, ret=True)
        try:
            value = int(values[1] << 8) + int(values[2])
            if values[5]:
                return value
            else:
                return -value
        except:
            return -1

    
    #   
    def execute(self, inst, params, ret=False):
        # self._ser.flushInput()
        self._mutex.acquire()  
        try:      
            self._ser.flushInput()
        except Exception as e:
            print e


        checksum = 0
        try:
            self._ser.write(chr(0xFF)+chr(0xFF)+chr(self._id)+chr(inst)+chr(len(params)))
        except Exception as e:
            print e
            self._mutex.release()
            return None
        for param in params:
            try:
                checksum += param
                self._ser.write(chr(param))
            except Exception as e:
                print e
                self._mutex.release()
                return None
        try:
            checksum = 255 - ((self._id + inst + len(params) + checksum) % 256)
            self._ser.write(chr(checksum))
        except Exception as e:
            print e
            self._mutex.release()
            return None
        # params = []
        if ret:
            params = self.paramGet(0)
        self._mutex.release()
        return params
        # print '0xFF', '0xFF', self._id, inst, len(params), params, checksum
    
    def paramGet(self, mode, id= -1, inst= -1, length= -1, params=None):
        d = self._ser.read()

        if d == '': 
            return None

        # print ord(d)
        # now process our byte
        if mode == 0:           # get our first 0xFF
            if ord(d) == 0xff:
                # print "Oxff 1 found"
                return self.paramGet(1)
            else:
                return self.paramGet(0)
        elif mode == 1:         # get our second 0xFF
            if ord(d) == 0xff:
                # print "Oxff 2 found"
                return self.paramGet(2)
        elif mode == 2:
            d = ord(d)
            if d != 0xff:       # read id
                return self.paramGet(3, d)
            else:
                return self.paramGet(0)
        elif mode == 3:         # read inst 
            return self.paramGet(4, id, ord(d))
        elif mode == 4:         # read length
            return self.paramGet(5, id, inst, ord(d))
        elif mode == 5:
            return self.paramGet(6, id, inst, length, list())
        elif mode == 6:
            params.append(ord(d))
            if len(params) == length:
                return self.paramGet(7, id, inst, length, params)
            return self.paramGet(6, id, inst, length, params)
        elif mode == 7:
            checksum = id + inst + length + sum(params) + ord(d)
            if checksum % 256 != 255:
                return None
            return params
        return None

    def convertToHalf(self, decimal):
        return int(decimal * 65278.0)

        

if __name__ == '__main__':
    pass
    # import time

    # controller = Controller(2, '/dev/ttyUSB0', 115200)
    
    
    # while True:
    #     # controller.setSpeed([SETPOINT1,SETPOINT4],[-65278,65278])
    #     controller.getSpeed([SETPOINT1,SETPOINT2])
    #     # time.sleep(.5)
