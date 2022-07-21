#!/usr/bin/env python3

import struct, time, binascii, datetime, sys

import serial
import crcmodbus

def debug(*args, **kwargs):
    if False:
        print(*args, **kwargs)
        sys.stdout.flush()

# Register definitions and examples from the Maynuo user manual, available
# from http://www.maynuo.com

class MaynuoLoad():

    CMDS = {
        'CC': 1,
        'CV': 2,
        'CW': 3,
        'CR': 4,
        'CC_Soft_Start': 20,
        'Dynamic_Mode': 25,
        'Short_Circuit_Mode': 26,
        'List_Mode': 27,
        'CC_Loading_And_Unloading_Mode': 30,
        'CV_Loading_And_Unloading_Mode': 31,
        'CW_Loading_And_Unloading_Mode': 32,
        'CR_Loading_And_Unloading_Mode': 33,
        'CC_Mode_Switch_To_CV_Mode': 34,
        'CR_Mode_Switch_To_CV_Mode': 36,
        'Battery_Test_Mode': 38,
        'CV_Soft_Start': 39,
        'Changing_System_Parameters':  41,
        'Input_ON': 42,
        'Input_OFF': 43,
    }


    REGS = {
        'CMD':       ( 0xa00,  1,  'H' ), # W  command register
        'IFIX':      ( 0xa01,  2,  'f' ), # W/R CC set register (A)
        'UFIX':      ( 0xa03,  2,  'f' ), # W/R CV set register (V)
        'PFIX':      ( 0xa05,  2,  'f' ), # W/R CP set register (W)
        'RFIX':      ( 0xa07,  2,  'f' ), # W/R CR set register (ohm)
        'TMCCS':     ( 0x0A09,  2, 'f' ), # W/R Current soft-start rising time register , double type
        'TMCVS':     ( 0x0A0B,  2, 'f' ), # W/R Voltage soft-start rising time register , double type
        'UCCONSET':  ( 0x0A0D,  2, 'f' ), # W/R Constant current load voltage register :double-type
        'UCCOFFSET': ( 0x0A0F,  2, 'f' ), # W/R constant current unload voltage register ， double-type
        'UCVONSET':  ( 0x0A11,  2, 'f' ), # W/R Constant voltage load voltage register :double-type
        'UCVOFFSET': ( 0x0A13,  2, 'f' ), # W/R Constant voltage unloaded voltage regi ， double-type
        'UCPONSET':  ( 0x0A15,  2, 'f' ), # W/R Constant power load voltage register，doubletype
        'UCPOFFSET': ( 0x0A17,  2, 'f' ), # W/R Constant power unload voltage register ，
        'UCRONSET':  ( 0x0A19,  2, 'f' ), # W/R Constant resistance load voltage register ， double-type
        'UCROFFSET': ( 0x0A1B,  2, 'f' ), # W/R Constant resistance unload voltage register， double type
        'UCCCV':     ( 0x0A1D,  2, 'f' ), # W/R constant current shift constant voltage register:double type
        'UCRCV':     ( 0x0A1F,  2, 'f' ), # W/R Constant resistance shift constant voltage register, double type
        'IA':        ( 0x0A21,  2, 'f' ), # W/R dynamic mode A phase current register, double-type
        'IB':        ( 0x0A23,  2, 'f' ), # W/R dynamic mode B phase current register, double-type
        'TMAWD':     ( 0x0A25,  2, 'f' ), # W/R dynamic mode A pulse-width registers, double-type
        'TMBWD':     ( 0x0A27,  2, 'f' ), # W/R dynamic mode B pulse-width registers ,double-type
        'TMTRANRIS': ( 0x0A29,  2, 'f' ), # W/R Dynamic mode rising time register,r double-type
        'TMTRANFAL': ( 0x0A2B,  2, 'f' ), # W/R Dynamic model falling time register double-type
        'MODETRAN':  ( 0x0A2D,  1, 'H' ), # W/R Dynamic mode register,u16-type
        'UBATTEND':  ( 0x0A2E,  2, 'f' ), # W/R Battery Test termination voltage register ,double type
        'BATT':      ( 0x0A30,  2, 'f' ), # W/R Battery capacity register, double –type
        'SERLIST':   ( 0x0A32,  1, 'H' ), # W/R LIST serial number register, u16 type
        'SERATEST':  ( 0x0A33,  1, 'H' ), # W/R Automatic Test serial number register ，u16 type
        'IMAX':      ( 0x0A34,  2, 'f' ), # W/R Current maximum register，double type
        'UMAX':      ( 0x0A36,  2, 'f' ), # W/R Voltage maximum register，double type
        'PMAX':      ( 0x0A38,  2, 'f' ), # W/R Power maximum register ,double type
        'ILCAL':     ( 0x0A3A,  2, 'f' ), # W/R Calibration current low-end target value double type
        'IHCAL':     ( 0x0A3C,  2, 'f' ), # W/R Current high-end calibration target value， double type
        'ULCAL':     ( 0x0A3E,  2, 'f' ), # W/R Voltage low-end calibration target value ， double type
        'UHCAL':     ( 0x0A40,  2, 'f' ), # W/R Voltage high-end calibration target value， double type
        'TAGSCAL':   ( 0x0A42,  1, 'H' ), # W/R Calibration state tag，u16 type
        'U':         ( 0x0B00,  2, 'f' ), # R Voltage Register ,double type
        'I':         ( 0x0B02,  2, 'f' ), # R Current Register ,double type
        'SETMODE':   ( 0x0B04,  1, 'H' ), # R Operation Mode register,u16e type
        'INPUTMODE': ( 0x0B05,  1, 'H' ), # R Input Status Register，u16 type
        'MODEL':     ( 0x0B06,  1, 'H' ), # R Model Register ,u16 type
        'EDITION':   ( 0x0B07,  1, 'H' ), # R software version number register,u16 type

    }

    COIL_FLAGS = {
        'PC1':        0x0500, # 1 W/R When remote control status bit is 1, front key panel unable
        'PC2':        0x0501, # 1 W/R When local prohibition bit is 1, not allow to 
                              # use key "Shift +7" to snatch away the front panel control.
        'TRIG':       0x0502, # 1 W/R Trigger tagged: triggered once by software
        'REMOTE':     0x0503, # 1 W/R 1: remote input voltage
        'ISTATE':     0x0510, # 1 R Input status: 1- input ON, 0- intput OFF
        'TRACK':      0x0511, # 1 R Tracking status: 1-voltage tracking; 0-current tracking
        'MEMORY':     0x0512, # 1 R 1:input state memory
        'VOICEEN':    0x0513, # 1 R 1: key sound ON/OFF
        'CONNECT':    0x0514, # 1 R 1: multi 0= single
        'ATEST':      0x0515, # 1 R 1: Automatic test mode
        'ATESTUN':    0x0516, # 1 R 1: Automatic test pattern waiting to trigger
        'ATESTPASS':  0x0517, # 1 R 1: success automatic test success ,0: automatic test failed
        'IOVER':      0x0520, # 1 R 1:over-current tag
        'UOVER':      0x0521, # 1 R 1: over-voltage tag
        'POVER':      0x0522, # 1 R 1: over- Power tag
        'HEAT':       0x0523, # 1 R 1: over-heat tag
        'REVERSE':    0x0524, # 1 R 1: reverse tag
        'UNREG':      0x0525, # 1 R 1: register parameter failed tag
        'ERREP':      0x0526, # 1 R 1: EPPROM error tag
        'ERRCAL':     0x0527, # 1 R 1: calibration data error tag 
    }
 
    def __init__(self, port = '/dev/ttyUSB0', baud=9600, slave_addr=1,
                 reg_write_delay=0.10):
        self.port = serial.Serial(port, baud, parity='N', timeout=1)
        self.slaveAddrB = slave_addr
        self.reg_write_delay = reg_write_delay

    #
    # --- private functions ---
    #


    def __addCRC(self, dataB):
        crc = crcmodbus.checksum(dataB)
        crcl = crc & 0xff
        crch = (crc >> 8) & 0xff
        return dataB + bytes([crch, crcl]) 

    def __checkWriteResponse(self, addr, func, len_or_val):
        readB = self.port.read(8)
        saddr, rfunc, raddr, rval, crc = struct.unpack('>BBHHH', readB)
        if saddr != self.slaveAddrB:
            raise Exception(f'Device slave address {saddr} does not match request {self.slaveAddrB}')
        if rfunc != func:
            raise Exception(f'Device return function {func} does not match request {rfunc}')
        if raddr != addr:
            raise Exception(f'Device return address {raddr} does not match written addr {addr}')
        if rval != len_or_val:
            raise Exception(f'Device return reg count {rval} does not match written reg count {len_or_val}')
        own_crc = crcmodbus.checksum(readB[:6])
        if own_crc != crc:
            raise Exception(f'Read CRC {crc} does not match expected {own_crc}')

    def __writeRegRaw(self, addr, dataB):
        debug('__writeRegRaw', self, addr, binascii.hexlify(dataB))

        dlen = len(dataB)
        wB = struct.pack('>BBHHB', self.slaveAddrB, 0x10, addr, int(dlen/2), dlen)
        wB += dataB
        wB = self.__addCRC(wB)
        debug('wB',binascii.hexlify(wB), len(wB))
        self.port.write(wB)
        time.sleep(self.reg_write_delay)
        self.__checkWriteResponse(addr, 0x10, int(dlen/2))

    def __writeCoilRaw(self, addr, v):
        wval = 0xff00 if v else 0x0000
        wB = struct.pack('>BBHH', self.slaveAddrB, 0x05, addr, wval)
        wB = self.__addCRC(wB)
        debug('__writeCoilRaw', 'wB', binascii.hexlify(wB))
        self.port.write(wB)
        time.sleep(self.reg_write_delay)
        self.__checkWriteResponse(addr, 0x05, wval)

        
    def __writeByName(self, name, val):
        debug('__writeByName', self, name, val)
        if isinstance(val, (bytes, bytearray)):
            return self.__writeRegRaw(self.REGS[name][0], val)
        return self.__writeRegRaw(self.REGS[name][0], struct.pack('>' + self.REGS[name][2], val))


    def __readGenericRaw(self, addr, source, dlen):
        debug('__readGenericRaw', self, addr, source, dlen)
        wB = bytearray((self.slaveAddrB, source))
        wB += struct.pack('>HH', addr, dlen)
        wB = self.__addCRC(wB)
        debug('__readGenericRaw wB',binascii.hexlify(wB), len(wB))
        self.port.write(wB)
        time.sleep(0.05)
        ret = self.port.read(3)
        debug('ret',binascii.hexlify(ret), len(ret))
        if int(ret[0]) != self.slaveAddrB:
            raise Exception(f'Device slave address {ret[0]} does not match request {self.slaveAddrB}')
        if int(ret[1]) != source:
            raise Exception(f'Device return function {ret[1]} does not match request {source}')
        readB = self.port.read(ret[2])
        (ret_crc,) = struct.unpack('>H',self.port.read(2))
        own_crc = crcmodbus.checksum(ret + readB)
        if own_crc != ret_crc:
            raise Exception(f'Read CRC {ret_crc} does not match expected {own_crc}')
        return readB

    def __writeCoil(self, name, val):
        try:
            addr = self.COIL_FLAGS.get(name)
        except Exception as e:
            raise Exception(f'unknown coil bit {name}')
        return self.__writeCoilRaw(addr, val)

    def __readCoil(self, name):
        try:
            addr = self.COIL_FLAGS.get(name)
        except Exception as e:
            raise Exception(f'unknown coil bit {name}')
        return self.__readGenericRaw(addr, 0x01, 1)


    def __readRegRaw(self, name, dlen):
        try:
            addr = self.REGS.get(name)[0]
        except Exception as e:
            raise Exception(f'unknown register {name}')

        return self.__readGenericRaw(addr, 0x03, dlen)


    #
    # --- public interface ---
    #


    # set a register to the provided val. val should be a float or
    # an int, as defined in the table of registers above
    def setReg(self, name, val):
        debug('setReg', self, name, val)
        return self.__writeByName(name, val)

    # return the contents of a register, appropriately converted
    def getReg(self, name):
        rinfo = self.REGS[name]
        if rinfo is None:
            raise Exception(f'unknown reg {name}')

        return struct.unpack('>' + rinfo[2], self.__readRegRaw(name, rinfo[1]))[0]

    # turn on the load
    # you can also turn it off with inputOn(False)
    def inputOn(self, on=True):
        cmd = self.CMDS['Input_ON' if on else 'Input_OFF']
        self.__writeRegRaw(self.REGS['CMD'][0], bytes((0x00, cmd)))
        
    # turn off the load
    def inputOff(self, off=True):
        return self.inputOn(not off) 
        
    def setCC(self, curr=0):
        self.__writeByName('IFIX',curr)
        self.__writeByName('CMD', bytes((0x00, self.CMDS['CC'])))

    def setCV(self, volt=0):
        self.__writeByName('VFIX', volt)
        self.__writeByName('CMD', bytes((0x00, self.CMDS['CV'])))

    def setCP(self, pwr=0):
        self.__writeByName('PFIX', pwr)
        self.__writeByName('CMD', bytes((0x00, self.CMDS['CW'])))

    def setCR(self, res=0):
        self.__writeByName('RFIX', res) 
        self.__writeByName('CMD', bytes((0x00, self.CMDS['CR'])))

    # turn on battery testing mode. This mode automatically
    # shuts off when the battery voltage goes below the
    # vend value. Total battery capacity is accumulated in
    # the BATT register, and this is *not* automatically zero'd
    # by this call. If you want it zero'd, do it directly
    def battTest(self, curr=0.250, vend=3.0):
        self.__writeByName('IFIX',curr)
        self.__writeByName('UBATTEND',vend)
        self.__writeByName('CMD', bytes((0x00, self.CMDS['Battery_Test_Mode'])))

    # a convenience accessor for getting the values of a few common registers
    def getSetupRegs(self):
        cmd, ifix, ufix, pfix, rfix = struct.unpack('>Hffff', self.__readRegRaw('CMD',9))
        return {
            'cmd': cmd,
            'ifix': ifix,
            'ufix': ufix,
            'pfix': pfix,
            'rfix': rfix,
        }

    # a convenience accessor for getting the values of the measurement registers
    def getOperatingPoint(self):
        ret = self.__readRegRaw('U', 4)
        (v,i) = struct.unpack('>ff', ret)
        bcap = self.getReg('BATT')
        return {
            'ts': datetime.datetime.now(),
            'v': v,
            'i': i,
            'bcap': bcap,
        }

    def getCoil(self, name):
        return True if self.__readCoil(name)[0] & 0x01 else False

    def setCoil(self, name, v):
        return self.__writeCoil(name, v)

     
if __name__ == '__main__':
    import sys
    m = MaynuoLoad('/dev/ttyUSB0', 9600, 1)

    if True:
        m.setReg('IFIX', 0.1)
        print(m.getReg('IFIX'))
        print(m.getOperatingPoint())
        m.setCoil('PC1', True)

    if True:
        for n in m.COIL_FLAGS:
            print(n, m.getCoil(n))
        for n in m.REGS:
            print(n, m.getReg(n))

    if True:
        for i in range(3):
            try:
                print(i, m.getOperatingPoint())
            except Exception as e:
                print(i, e)

    if True:
        m.setReg('BATT', 0)
        m.inputOff()
        m.battTest(0.150, 3)
        m.inputOn()
        for i in range(4):
            try:
                op = m.getOperatingPoint()
                print(i,op)
            except Exception as e:
                print(i,e)
            time.sleep(1)
        m.inputOff()
        print(m.getOperatingPoint())

    if True:
        for n in m.COIL_FLAGS:
            print(n, m.getCoil(n))
        for n in m.REGS:
            print(n, m.getReg(n))
