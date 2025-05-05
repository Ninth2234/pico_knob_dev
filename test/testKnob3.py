from config import board_pico_drive_03c as board
from bldc_driver import BLDCDrive,normalize_angle_signed
from AS5048a import AS5048A
from machine import reset
import time
from gc9a01py import GC9A01,color565

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

        self._old_angle = current_angle
        self._old_t = t
        return effort
    

    
class PulseGen_Controller(Controller):

    def __init__(self):
        self.k_p = 0.03
        self.k_d = 400
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


non_controller = Position_controller()
pulseGen_Controller = PulseGen_Controller()


from array import array
from color_setup import ssd  # Create a display instance

from gui.core.nanogui import refresh
from gui.core.writer import CWriter

import asyncio
from gui.core.colors import *
import gui.fonts.arial35 as font
from gui.widgets.label import Label
from gui.widgets.scale import Scale

import time
import math

def transform_poly(coords, tx, ty, angle_deg=0, scale=1.0):
    """Transform polygon by translation, rotation, and scaling.

    Args:
        coords: array('h') of [x0, y0, x1, y1, ..., xn, yn]
        tx, ty: translation
        angle_deg: rotation angle in degrees
        scale: uniform scale factor

    Returns:
        list of transformed (x, y) tuples
    """
    result = []
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)

    for i in range(0, len(coords), 2):
        x = coords[i] * scale
        y = coords[i + 1] * scale

        # Apply rotation
        xr = x * cos_a - y * sin_a
        yr = x * sin_a + y * cos_a

        # Apply translation
        xr += tx
        yr += ty
        
        result.append(round(xr))
        result.append(round(yr))  # Use int for framebuffer drawing

    return array('h',result)

BG_COLOR = BLACK

CENTER = (120,110)
RING_OUTER_RADIUS = 80
RING_WIDTH = 2

refresh(ssd,True)

def draw_ring():
    ssd.ellipse(*CENTER,RING_OUTER_RADIUS,RING_OUTER_RADIUS,GREEN,True)
    ssd.ellipse(*CENTER,RING_OUTER_RADIUS-RING_WIDTH,RING_OUTER_RADIUS-RING_WIDTH,BG_COLOR,True)


width = 2
length = 5

shape1 = array('h',[
    -width,-length,
    +width,-length,
    +width,+length,
    -width,+length,
])


writer = CWriter(ssd, font, WHITE)

r = 80

import _thread

def task_motor():
    motor.enable()
    while True:
        start = time.ticks_us()
        encoder.update()
        effort = pulseGen_Controller.compute(encoder.getAngle())
        motor.drive(effort)
        end = time.ticks_us()
        # print("Motor loop time (us):", time.ticks_diff(end, start))
        time.sleep_ms(0)


def task_screen():
    
    while True:
        angle = round(0-encoder.getAngle())%360
        draw_screen(angle)
        time.sleep_ms(2)  # 50 FPS max

def draw_screen(angle):
    ssd.fill(0)
    draw_ring()
    angle_rad = math.radians(angle-90)
    x = r * math.cos(angle_rad)
    y = r * math.sin(angle_rad)
    writer.set_textpos(ssd, 95,105)
    writer.printstring(str(round(angle/10)))
    t_shape1 = transform_poly(shape1, x, y, angle)
    ssd.poly(*CENTER, t_shape1, WHITE, True)
    refresh(ssd)

    
# Start screen drawing on second core
_thread.start_new_thread(task_screen, ())

# Run motor task on main core
try:
    # task_screen()
    task_motor()
finally:
    motor.disable()
    
    
