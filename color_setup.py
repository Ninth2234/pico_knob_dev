from machine import Pin, SPI
import gc
from drivers.gc9a01.gc9a01 import GC9A01 as SSD

from drivers.gc9a01.gc9a01_8_bit import GC9A01 as SSD

from config import board_pico_drive_03c as board


gc.collect()  # Precaution before instantiating framebuf

ssd = SSD(board.screen_spi, dc=board.screen_dc, cs=board.screen_cs, rst=board.screen_reset, lscape=False, usd=False, mirror=False)
