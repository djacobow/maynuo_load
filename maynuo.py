#!/usr/bin/env python3

import struct, time, binascii, datetime

import serial
import crcmodbus

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

    def __init__(self, port = '/dev/ttyUSB0', baud=9600, slave_addr=1,
                 reg_write_delay=0.10):
        self.port = serial.Serial(port, baud, parity='N', timeout=1)
        self.slaveAddrB = slave_addr
        self.reg_write_delay = reg_write_delay

    #
    # --- private functions ---
    #


    def _addCRC(self, dataB):
        crc = crcmodbus.checksum(dataB)
        crcl = crc & 0xff
        crch = (crc >> 8) & 0xff
        return dataB + bytes([crch, crcl]) 

    def _writeReg(self, addr, dataB):
        wB = bytearray([self.slaveAddrB])
        addr = struct.pack('>H',addr)
        dlen = len(dataB)
        wB += bytes([0x10]) + addr + bytes([0x00, int(dlen/2), dlen])
        wB += dataB
        wB = self._addCRC(wB)
        # print('wB',binascii.hexlify(wB), len(wB))
        self.port.write(wB)
        time.sleep(self.reg_write_delay)

    def _writeByName(self, name, val):
        if isinstance(val, (bytes, bytearray)):
            return self._writeReg(self.REGS[name][0], val)
        return self._writeReg(self.REGS[name][0], struct.pack('>' + self.REGS[name][2], val))


    def _readRegRaw(self, name, dlen):
        try:
            addr = self.REGS.get(name)[0]
        except Exception as e:
            raise Exception(f'unknown register {name}')

        wB = bytearray([self.slaveAddrB])
        wB += bytes([0x03]) + struct.pack('>HH', addr, dlen)
        self.port.flushInput()
        wB = self._addCRC(wB)
        # print('wB',binascii.hexlify(wB), len(wB))
        self.port.write(wB)
        #time.sleep(self.reg_write_delay)
        ret = self.port.read(3)
        # print('ret',binascii.hexlify(ret), len(ret))
        if int(ret[0]) != self.slaveAddrB:
            raise Exception(f'Device slave address {ret[0]} does not match request {self.slaveAddrB}')
        if int(ret[1]) != 0x03:
            raise Exception(f'Device return function does not match request 0x03')
        ret_len        = ret[2]
        readB = self.port.read(ret_len)
        (ret_crc,) = struct.unpack('>H',self.port.read(2))
        own_crc = crcmodbus.checksum(ret + readB)
        if own_crc != ret_crc:
            raise Exception(f'Read CRC {ret_crc} does not match expected {own_crc}')
        # print('readB',binascii.hexlify(readB), len(readB))
        return readB

    #
    # --- public interface ---
    #


    # set a register to the provided val. val should be a float or
    # an int, as defined in the table of registers above
    def setReg(self, name, val):
        return self._writeByName(name, val)

    # return the contents of a register, appropriately converted
    def getReg(self, name):
        rinfo = self.REGS[name]
        if rinfo is None:
            raise Exception(f'unknown reg {name}')

        return struct.unpack('>' + rinfo[2], self._readRegRaw(name, rinfo[1]))[0]

    # turn on the load
    # you can also turn it off with inputOn(False)
    def inputOn(self, on=True):
        cmd = self.CMDS['Input_ON' if on else 'Input_OFF']
        self._writeReg(self.REGS['CMD'][0], bytes((0x00, cmd)))
        
    # turn off the load
    def inputOff(self, off=True):
        return self.inputOn(not off) 
        
    def setCC(self, curr=0):
        self._writeByName('IFIX',curr)
        self._writeByName('CMD', bytes((0x00, self.CMDS['CC'])))

    def setCV(self, volt=0):
        self._writeByName('VFIX', volt)
        self._writeByName('CMD', bytes((0x00, self.CMDS['CV'])))

    def setCP(self, pwr=0):
        self._writeByName('PFIX', pwr)
        self._writeByName('CMD', bytes((0x00, self.CMDS['CW'])))

    def setCR(self, res=0):
        self._writeByName('RFIX', res) 
        self._writeByName('CMD', bytes((0x00, self.CMDS['CR'])))

    # turn on battery testing mode. This mode automatically
    # shuts off when the battery voltage goes below the
    # vend value. Total battery capacity is accumulated in
    # the BATT register, and this is *not* automatically zero'd
    # by this call. If you want it zero'd, do it directly
    def battTest(self, curr=0.250, vend=3.0):
        self._writeByName('IFIX',curr)
        self._writeByName('UBATTEND',vend)
        self._writeByName('CMD', bytes((0x00, self.CMDS['Battery_Test_Mode'])))

    # a convenience accessor for getting the values of a few common registers
    def getSetupRegs(self):
        cmd, ifix, ufix, pfix, rfix = struct.unpack('>Hffff', self._readRegRaw('CMD',9))
        return {
            'cmd': cmd,
            'ifix': ifix,
            'ufix': ufix,
            'pfix': pfix,
            'rfix': rfix,
        }

    # a convenience accessor for getting the values of the measurement registers
    def getOperatingPoint(self):
        ret = self._readRegRaw('U', 4)
        (v,i) = struct.unpack('>ff', ret)
        bcap = self.getReg('BATT')
        return {
            'ts': datetime.datetime.now(),
            'v': v,
            'i': i,
            'bcap': bcap,
        }

     
if __name__ == '__main__':
    m = MaynuoLoad('/dev/ttyUSB0', 9600, 1)

    m.setReg('BATT', 0)
    m.inputOff()
    if True:
        m.battTest(0.150, 3)
        m.inputOn()
        for i in range(5):
            print(m.getOperatingPoint())
            time.sleep(5)
        m.inputOff()
        m.inputOff()
        print(m.getOperatingPoint())

    for n in m.REGS:
        print(n, m.getReg(n))
