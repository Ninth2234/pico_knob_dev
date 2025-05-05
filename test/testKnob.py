from config import board_pico_drive_03c as board
from bldc_driver import BLDCDrive,normalize_angle_signed
from AS5048a import AS5048A
import time

encoder = AS5048A(board.as5048a_spi,board.as5048a_cs)
motor = BLDCDrive(board.pwm_u,board.pwm_v,board.pwm_w,board.motor_enable,reverse=True)
motor.attach_encoder(encoder.getAngle)

class LowPass:
    def __init__(self, alpha):
        self.a = alpha
        self.y = 0

    def filter(self, x):
        self.y = self.a * x + (1 - self.a) * self.y
        return self.y


class Controller:
    def __init__(self):

        ...

    def compute(self, current_angle):

        raise NotImplementedError("Override this method in a subclass.")



class Position_controller(Controller):

    def __init__(self):
        self.k_p = 0
        self.k_d = 1000
        self.ref = 0
        
        self._old_angle = 0
        self._old_t = time.ticks_us()
        
        self.t_alpha = 500

        self.vel_lowpass = LowPass(0.02)
        

    def compute(self, current_angle):
        t = time.ticks_us()
        error = normalize_angle_signed(self.ref-current_angle)
        
        vel = normalize_angle_signed(current_angle-self._old_angle)/(t-self._old_t+self.t_alpha)
        vel = self.vel_lowpass.filter(vel)
        effort = self.k_p*error - self.k_d*vel

        self._old_angle = current_angle
        self._old_t = t
        return effort
    
class PulseGen_Controller(Controller):

    def __init__(self):
        self.k_p = 0.03
        self.k_d = 500
        self.ref = 0
        
        self._old_angle = 0
        self._old_t = time.ticks_us()
        
        self.t_alpha = 500

        self.vel_lowpass = LowPass(0.02)

        self.delta = 10
        

    def compute(self, current_angle):
        self.ref = self.pulse_generator(self.ref,current_angle)

        t = time.ticks_us()
        error = normalize_angle_signed(self.ref-current_angle)
        
        vel = normalize_angle_signed(current_angle-self._old_angle)/(t-self._old_t+self.t_alpha)
        vel = self.vel_lowpass.filter(vel)
        effort = self.k_p*error - self.k_d*vel

        self._old_angle = current_angle
        self._old_t = t
        return effort
    
    def pulse_generator(self, ref_angle, angle):
        # Calculate the difference between the current angle and the reference
        diff = normalize_angle_signed(angle - ref_angle)
        
        # If the angle difference crosses half the delta, adjust the reference angle
        if diff > self.delta / 2:
            ref_angle += self.delta
        elif diff < -self.delta / 2:
            ref_angle -= self.delta

        return ref_angle


pos_controller = Position_controller()
pulseGen_Controller = PulseGen_Controller()



try:
    motor.enable()
    while True:
        encoder.update()

        # effort = pulseGen_Controller.compute(encoder.getAngle())
        effort = 1
        motor.drive(effort)
        print(effort)        
        # time.sleep(0.1)

finally:
    motor.disable()


    
