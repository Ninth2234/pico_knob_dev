from machine import Pin, SPI
import time
from AS5048a import AS5048A
from config import board_pico_drive_03c as pin
from config.board_pico_drive_03c import as5048a_spi


encoder = AS5048A(as5048a_spi,pin.as5048a_cs)

# Example usage



tasks = [
    ("read spi",encoder.update),
    ("get data",encoder.getAngle)
]
MAXIMUM_RAW_ANGLE = const(2**14)

while True:
    
    for [name,task] in tasks:
        start = time.ticks_us()
        task()
        end = time.ticks_us()
        elapsed = time.ticks_diff(end, start)  # handles wraparound safely
        print(name,"Elasped time:", elapsed)

    start = time.ticks_us()
    tmp = encoder.rawAngle
    end = time.ticks_us()
    elapsed = time.ticks_diff(end, start)  # handles wraparound safely
    print(tmp*360/MAXIMUM_RAW_ANGLE)
    print("direct access","Elasped time:", elapsed)
    # print("Response:", response)
    time.sleep(1)
