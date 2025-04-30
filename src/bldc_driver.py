import math
from machine import PWM


PI = math.pi
TWO_PI = 2*PI
PI_2 = math.pi / 2
PI_3 = math.pi / 3
DEG_2_RAD = math.pi/180
SQRT3 = math.sqrt(3)
DIV_SQRT3 = 1/SQRT3

def normalize_angle(angle):
    return angle % (2 * PI)

def normalize_angle_signed(x):
    return (x + 180)%360 - 180

class BLDCDrive:
    def __init__(self,pwm_u:PWM,pwm_v:PWM,pwm_w:PWM,enable_pin,reverse=False):
        self.pwm_u = pwm_u
        if reverse:
            self.pwm_v = pwm_w
            self.pwm_w = pwm_v
        else:
            self.pwm_v = pwm_v
            self.pwm_w = pwm_w
        self.ena_pin = enable_pin

    def attach_encoder(self, func_read_encoder):
        """
        Attach an external encoder reading function.

        Args:
            func_read_encoder (Callable[[], float]): A function that returns the current mechanical angle [rad].
        """
        self.func_read_encoder = func_read_encoder

    def drive(self, effort):
        """
        Drive the motor using feedback from the attached encoder.

        Args:
            effort (float): Control effort in the range [-1.0, 1.0].x
        """
        effort = max(min(effort, 1.0), -1.0)
        self._drive(effort, self.func_read_encoder())

    def _drive(self, effort, angle):        
        """
        Drive the motor using a manually provided angle (open-loop or custom feedback).

        Args:
            effort (float): Control effort in the range [-1.0, 1.0].
            angle (float): Mechanical angle [rad] used to compute electrical phase.
        """
        self.setPhaseVoltage(effort, self.cal_el_angle(angle))

    def disable(self):
        self.pwm_u.duty_u16(0)
        self.pwm_v.duty_u16(0)
        self.pwm_w.duty_u16(0)
        self.ena_pin.low()
        
    def enable(self):
        self.ena_pin.high()

    def setPwm(self, ua, ub, uc):
        self.pwm_u.duty_u16(int(ua*65535))
        self.pwm_v.duty_u16(int(ub*65535))
        self.pwm_w.duty_u16(int(uc*65535))
    
    def setPhaseVoltage(self, effort, angle_el):
        
        """_summary_

        Args:
            Uq (float): input effort [-1 1]
            angle_el (_type_): _description_
        """
        if effort < 0:
            angle_el += PI
            effort = -effort            
        
        effort = min(effort,1)

        angle_el = normalize_angle(angle_el + PI_2)
        sector = int(angle_el / PI_3) + 1

        T1 = math.sin(sector * PI_3 - angle_el) * effort
        T2 = math.sin(angle_el - (sector - 1.0) * PI_3) * effort
        T0 = 1 - T1 - T2

        if sector == 1:
            Ta = T1 + T2 + T0 / 2
            Tb = T2 + T0 / 2
            Tc = T0 / 2
        elif sector == 2:
            Ta = T1 + T0 / 2
            Tb = T1 + T2 + T0 / 2
            Tc = T0 / 2
        elif sector == 3:
            Ta = T0 / 2
            Tb = T1 + T2 + T0 / 2
            Tc = T2 + T0 / 2
        elif sector == 4:
            Ta = T0 / 2
            Tb = T1 + T0 / 2
            Tc = T1 + T2 + T0 / 2
        elif sector == 5:
            Ta = T2 + T0 / 2
            Tb = T0 / 2
            Tc = T1 + T2 + T0 / 2
        elif sector == 6:
            Ta = T1 + T2 + T0 / 2
            Tb = T0 / 2
            Tc = T1 + T0 / 2
        else:
            Ta = Tb = Tc = 0

        Ua = Ta
        Ub = Tb
        Uc = Tc
        # print(Ua,Ub,Uc)
        self.setPwm(Ua, Ub, Uc)
    
    def cal_el_angle(self,angle):

        res = (angle-300.8496)*7
        return normalize_angle(res/180*PI)
    

        
## TODO
def calibrate(nPass=7):
    MAX_ANGLE = 360

    EFFORT = 0.8
    

    for i in range(360):
        el_angle = i*PI/180
        encoder.update()
        bldc.setPhaseVoltage(EFFORT,el_angle-PI_2)

    for _ in range(nPass):
        for i in range(360):
            el_angle = i*PI/180
            encoder.update()
            bldc.setPhaseVoltage(EFFORT,el_angle-PI_2)
            sleep(0.01)
        print(encoder.getAngle(),el_angle)

    



if __name__ == "__main__":
    from config import board_pico_drive_03c as pin
    from time import sleep
    from AS5048a import AS5048A

    encoder = AS5048A(pin.as5048a_spi,pin.as5048a_cs)


    
    try:
        pin.motor_enable.on()
        bldc = BLDCDrive(pin.pwm_u,pin.pwm_v,pin.pwm_w,pin.motor_enable,reverse=True)
        # while True:
        #     for i in range(360):
        #         el_angle = i*PI/180
        #         encoder.update()
        #         print(encoder.getAngle(),el_angle,bldc.cal_el_angle(encoder.getAngle()))
                # bldc.setPhaseVoltage(0.6,el_angle-PI_2)
        bldc.attach_encoder(encoder.getAngle)
        while True:
            encoder.update()
            
            bldc.drive(0.1)
        #     sleep(0.1)
        # calibrate()                
                      
                
    finally:
        pin.motor_enable.off()