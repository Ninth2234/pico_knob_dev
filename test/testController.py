
from config import board_pico_drive_03c as board
from bldc_driver import BLDCDrive,normalize_angle_signed
from AS5048a import AS5048A
import time
from gc9a01py import GC9A01,color565
import micropython

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

@micropython.native
def deadband(x, threshold):
    
    if -threshold < x and x < threshold:
        return 0
    return x

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
        
        self.t_alpha = 0

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

class Wall_Controller(Controller):

    def __init__(self):
        self.k_p = 0
        self.k_p2 = 0.1
        self.k_d = 500
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

        if current_angle<90:
            effort = normalize_angle_signed(90-current_angle)*self.k_p2
        elif current_angle>180:
            effort = normalize_angle_signed(180-current_angle)*self.k_p2

        self._old_angle = current_angle
        self._old_t = t
        return effort

class PulseGen2_Controller(Controller):

    def __init__(self):
        self.k_p = 0
        self.k_d = 500
        self.ref = 0
        
        self._old_angle = 0
        self._old_t = time.ticks_us()
        
        self.t_alpha = 0

        self.vel_lowpass = LowPass(0.05)

        self.delta = 10

        self.y = 0
        self.y2 = 15

        self.k_fs = 0.01
        self.fs_0 = 0

        self.x_width = 20
        self.x_half_width = self.x_width/2

        self.x_transit = 8
    
    def _cal_y(self,x):

        xi = x%self.x_width

        xi_half = xi%self.x_half_width

        y = (xi_half-(self.x_half_width-self.x_transit))*self.y2/self.x_transit

        y = max(y,0)
        
        if xi>self.x_half_width:
            y = self.y2-y
            self.y = y
            return y,-1
        self.y = y
        return y,1
    


    def compute(self, current_angle):
        

        t = time.ticks_us()
        
        vel = normalize_angle_signed(current_angle-self._old_angle)/(t-self._old_t+self.t_alpha)
        
        vel = self.vel_lowpass.filter(vel)
        
        
        effort = -self.k_d*vel
        
        
        y,dir = self._cal_y(current_angle)
        
        vel = deadband(vel,1e-5)
        if vel>0:
            effort-=self.fs_0
        elif vel<0:
            effort+=self.fs_0
        
        if self.y2 != y:
            effort -= dir*y*self.k_fs
        elif self.y2 == y:
            effort+= -1000*vel
        
        
        # effort += self.k_fs*y

        


        
        

        self._old_angle = current_angle
        self._old_t = t
        return effort

def smooth_deadband(x: float, width: float) -> float:
    if abs(x) <= width:
        return 0.0
    elif x > width:
        d = x - width
        return (d**3) / (width**2) + width / 2
    else:  # x < -width
        d = -x - width
        return -((d**3) / (width**2) + width / 2)


class PulseGen_Controller3(Controller):
    def __init__(self):
        self.k_p = 0.04
        self.k_d = 500
        self.ref = 0
        
        self._old_angle = 0
        self._old_t = time.ticks_us()
        
        self.t_alpha = 0

        self.vel_lowpass = LowPass(0.02)

        self.delta = 20

    def compute(self, current_angle):
        self.ref = self.pulse_generator(self.ref,current_angle)

        t = time.ticks_us()
        error = normalize_angle_signed(self.ref-current_angle)
        


        vel = normalize_angle_signed(current_angle-self._old_angle)/(t-self._old_t+self.t_alpha)
        vel = self.vel_lowpass.filter(vel)

        if error>0:
            error = max(error-0.5,0)
        else:
            error = min(error+0.5,0)

        if abs(error)>8:
            error = 0

        

        effort = self.k_p*error - self.k_d*vel

        if current_angle >270:
            effort = 0.1*(270-current_angle)
        
        if current_angle <90:
            effort = 0.1*(90-current_angle)

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

wall_controller = Wall_Controller()

pos_controller = Position_controller()
pulseGen_Controller = PulseGen_Controller()

pulseGen2_Controller = PulseGen2_Controller()

pulseGen3_controller = PulseGen_Controller3()






try:
    motor.enable()
    cal_control = pulseGen3_controller.compute
    drive = motor.drive
    while True:
        start = time.ticks_us()
        encoder.update()
        
        effort = cal_control(encoder.getAngle())
        
        drive(effort)
        end = time.ticks_us()
        elapsed = time.ticks_diff(end, start)  # handles wraparound safely
        print("Elasped time:", elapsed, effort, pulseGen2_Controller.y)
        

finally:
    motor.disable()


    
