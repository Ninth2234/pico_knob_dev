from gc9a01py import GC9A01,color565
from config import board_pico_drive_03c as board
import time
import math
from machine import SPI

screen_spi      = SPI(1, baudrate=10_000_000, sck=board.screen_sck, mosi=board.screen_mosi)
tft = GC9A01(
    screen_spi,
    dc=board.screen_dc,
    cs=board.screen_cs,
    reset=board.screen_reset,
    backlight=board.screen_blk,
    rotation=0)
tft.fill(0)
WIDTH = tft.width
HEIGHT = tft.height

WHITE = color565(255,255,255)
NON = color565(0,0,0)

x0 = 0
y0 = 0
x1 = 120
y1 = 120
tft.line(x0,y0,x1,y1,WHITE)

DEG_2_RAD = 3.14/180



x_cen = 120
y_cen = 120


x = 0
y = 0

now = time.ticks_ms()
for theta in range(360):
    print(time.ticks_ms()-now)
    now = time.ticks_ms()
    x_old = x
    y_old = y
    
    x = 120*math.cos(theta*DEG_2_RAD)+x_cen
    y = 120*math.sin(theta*DEG_2_RAD)+y_cen
    
    
    tft.fill(0)
    
    print(time.ticks_ms()-now)
    now = time.ticks_ms()
    tft.line(int(x1),int(y1),int(x_old),int(y_old),WHITE)
    




