
import micropython
from machine import Pin, SPI
from micropython import const
import time

ANGLE_REG = const(0x3FFF)

MAXIMUM_RAW_ANGLE = const(2**14)

class RotaryEncoder:
    def __init__(self,reverse = False):
        self.rawAngle = 0
        self._hardwareAngle = 0
        if reverse:
            self.reverse = -1
        else:
            self.reverse = 1

    def read_raw_angle(self):
        raise NotImplementedError("Subclasses should implement this!")
    
    def _getMaximumValue(self):
        raise NotImplementedError("Subclasses should implement this!")

    @micropython.native
    def update(self):
        
        newHardwareAngle = self.read_raw_angle()
        
        self.rawAngle += self.reverse*(newHardwareAngle - self._hardwareAngle)
        self._hardwareAngle = newHardwareAngle
        

    def getRawAngle(self):
        return self.rawAngle
    
    @micropython.native
    def getAngle(self)->float:
        return self.rawAngle*360/self._getMaximumValue()
    
    def getHardwareAngle(self):
        return self._hardwareAngle/self._getMaximumValue()*360

class AS5048A(RotaryEncoder):
    def __init__(self, spi_bus, cs_pin, maximumRawAngle = MAXIMUM_RAW_ANGLE,reverse = False):
        super().__init__(reverse)
        self.cs = cs_pin
        self.spi = spi_bus
        self.maximumRawAngle = maximumRawAngle

        # Set CS high to start with
        self.cs.high()
        

    # REGISTER LIST
    @micropython.native
    def read_angle_register(self):

        self.cs.low()
        receiveData = self.spi.read(2,ANGLE_REG)
        self.cs.high()
        
        rawAngle = (((receiveData[0]&0x3F)<<8)|receiveData[1])
        
        return rawAngle 
    
    @micropython.native
    def read_raw_angle(self)->int:
        response = self.read_angle_register()
        angle = response & 0x3FFF
        return angle
    
    @micropython.native
    def _getMaximumValue(self)->int:
        return self.maximumRawAngle
    
    def _read_register(self,reg):
        self.cs.low()
        recv = self.spi.read(2,reg)
        self.cs.high()
        return recv
    

    



    





# Example usage:
if __name__ == "__main__":

    spi_bus = SPI(id = 0,baudrate=4_000_000, sck=Pin(2), mosi=Pin(3), miso=Pin(0), polarity=0, phase=1)
    chipSelect = Pin(1,Pin.OUT)
    as5048a = AS5048A(spi_bus, chipSelect)

    while True:
        
        # as5048a.update()
        # now = time.ticks_us()
        as5048a.update()
        # after = time.ticks_us()
        reg_3FFD = as5048a._read_register(0x3FFD)
        reg_3FFE = as5048a._read_register(0x3FFE)

        # Print 0x3FFD as binary string (16 bits)
        # print("0x3FFD:", "{0:016b}".format(reg_3FFD))

        # Print 0x3FFE as integer
        print("Diag:",reg_3FFD,"\t", (reg_3FFD[0]<<8)|reg_3FFD[1])
        print("Gain:",reg_3FFD,"\t", (reg_3FFD[1]))
        print("Mag:",reg_3FFE,"\t", (reg_3FFE[0]<<8)|reg_3FFE[1])


        # angle = as5048a.getAngle()
        # rawAngle = as5048a.read_raw_angle()
        # print("Angle2:", angle, rawAngle, after-now)
        
        time.sleep(1)
