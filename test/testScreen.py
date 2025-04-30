"""
hello.py

    Writes "Hello!" in random colors at random locations on the display

"""
import random
from machine import Pin, SPI
import gc9a01py as gc9a01

# Choose a font

# from fonts import vga1_8x8 as font
# from fonts import vga2_8x8 as font
# from fonts import vga1_8x16 as font
# from fonts import vga2_8x16 as font
# from fonts import vga1_16x16 as font
# from fonts import vga1_bold_16x16 as font
# from fonts import vga2_16x16 as font
# from fonts import vga2_bold_16x16 as font
# from fonts import vga1_16x32 as font
# from fonts import vga1_bold_16x32 as font
# from fonts import vga2_16x32 as font
from fonts.bitmap import vga2_bold_16x32 as font

if __name__ == "__main__":
    from config import board_pico_drive_03c as pin
    from config.board_pico_drive_03c import screen_spi

    # spi = SPI(1, baudrate=1_000_000, sck=Pin(14), mosi=Pin(15))
    tft = gc9a01.GC9A01(
        screen_spi,
        dc=pin.screen_dc,
        cs=pin.screen_cs,
        reset=pin.screen_reset,
        backlight=pin.screen_blk,
        rotation=0)

    while True:
        for rotation in range(8):
            tft.rotation(rotation)
            tft.fill(0)
            col_max = tft.width - font.WIDTH*6
            row_max = tft.height - font.HEIGHT

            for _ in range(25):
                tft.fill(0)
                tft.text(
                    font,
                    "Hello!",
                    random.randint(0, col_max),
                    random.randint(0, row_max),
                    gc9a01.color565(
                        random.getrandbits(8),
                        random.getrandbits(8),
                        random.getrandbits(8)),
                    gc9a01.color565(
                        random.getrandbits(8),
                        random.getrandbits(8),
                        random.getrandbits(8))
                )


