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

CENTER = (120,120)
RING_OUTER_RADIUS = 110
RING_WIDTH = 2

refresh(ssd,True)

def draw_ring():
    ssd.ellipse(*CENTER,RING_OUTER_RADIUS,RING_OUTER_RADIUS,GREEN,True)
    ssd.ellipse(*CENTER,RING_OUTER_RADIUS-RING_WIDTH,RING_OUTER_RADIUS-RING_WIDTH,BG_COLOR,True)


width = 5
length = 2

shape1 = array('h',[
    -width,-length,
    +width,-length,
    +width,+length,
    -width,+length,
])


writer = CWriter(ssd, font, WHITE)

r = 80

def draw_screen(angle):
    ssd.fill(0)
    draw_ring()
    angle_rad = math.radians(angle)
    x,y = r*math.cos(angle_rad),r*math.sin(angle_rad)
    writer.set_textpos(ssd,*CENTER)
    writer.printstring(str(angle))
    t_shape1 = transform_poly(shape1,x,y,angle)
    # ssd.text(str(angle),*CENTER,WHITE)
    ssd.poly(*CENTER,t_shape1,WHITE,True)
    refresh(ssd)

for angle in range(360):
    draw_screen(angle)
    
    
    


# ssd.poly(*CENTER,shape2,GREEN,True)




