

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
        self.k_p = 0.01
        self.k_d = 500
        self.ref = 0.0
        
        self._old_angle = 0
        self._old_t = time.ticks_us()
        
        self.t_alpha = 0

        self.vel_lowpass = LowPass(0.02)

        self.delta = 10
        self.ticks = 0
        

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
            self.ticks+=1
        elif diff < -self.delta / 2:
            ref_angle -= self.delta
            self.ticks-=1

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
    
    

wall_controller = Wall_Controller()

pos_controller = Position_controller()
pulseGen_Controller = PulseGen_Controller()

pulseGen2_Controller = PulseGen2_Controller()

scaleIdx = 0
scales = [1,10,25]
from machine import Pin
last_pressed = 0  # timestamp of last valid press
debounce_ms = 200  # debounce threshold

def change_scaling(pin):
    global scaleIdx, last_pressed
    now = time.ticks_ms()
    if time.ticks_diff(now, last_pressed) > debounce_ms:
        last_pressed = now
        scaleIdx = (scaleIdx + 1) % len(scales)
        print("CHANGE TO", scales[scaleIdx])
    
board.button.irq(trigger=Pin.IRQ_FALLING,handler=change_scaling)

is_running = True

from color_setup import ssd  # Create a display instance

from gui.core.nanogui import refresh
from gui.core.writer import CWriter

import asyncio
from gui.core.colors import *
import gui.fonts.arial35 as font
from gui.widgets.label import Label
from gui.widgets.scale import Scale

CENTER = (120,120)
RING_OUTER_RADIUS = 80
RING_WIDTH = 3
writer = CWriter(ssd, font, WHITE)

COLORS = [RED,BLUE,GREEN]
def draw_screen(angle):
    ssd.fill(0)
    ssd.ellipse(*CENTER,RING_OUTER_RADIUS,RING_OUTER_RADIUS,COLORS[scaleIdx],True)
    ssd.ellipse(*CENTER,RING_OUTER_RADIUS-RING_WIDTH,RING_OUTER_RADIUS-RING_WIDTH,BLACK,True)
    writer.set_textpos(ssd,*CENTER)
    writer.printstring(str(angle))
    writer.set_textpos(ssd,CENTER[0]+40,CENTER[1])
    writer.printstring("x"+str(scales[scaleIdx]))
    refresh(ssd)


def task_send_command():
    current_ticks =  pulseGen_Controller.ticks
    old_ticks = current_ticks
    angle_command = 0
    angle_command += (current_ticks-old_ticks)*scales[scaleIdx]
    last_time = time.ticks_ms()
    while is_running:
        current_ticks =  pulseGen_Controller.ticks
        angle_command += (current_ticks-old_ticks)*scales[scaleIdx]
        uart.write(str(angle_command)+'\n')
        old_ticks = current_ticks
        time.sleep_ms(3)
        if time.ticks_ms()-last_time > 100:
            draw_screen(angle_command)
            last_time = time.ticks_ms()

from machine import UART,Pin
import _thread

uart = UART(1,tx=Pin(4),rx=Pin(5),baudrate=1_000_000)
_thread.start_new_thread(task_send_command, ())
try:
    motor.enable()
    encoder.update()
    # pulseGen_Controller.ref = encoder.getAngle()
    while True:
        
        encoder.update()
        
        
        effort = pulseGen_Controller.compute(encoder.getAngle())
        # # effort = pulseGen2_Controller.compute(encoder.getAngle())
        # start = time.ticks_us()
        motor.drive(effort)
        # # # print(effort,encoder.getAngle())
        
        # end = time.ticks_us()
        # elapsed = time.ticks_diff(end, start)  # handles wraparound safely
        # print("Elasped time:", elapsed)
        
        

finally:
    motor.disable()
    is_running = False

    
