from machine import Pin, SPI
from time import sleep
from AS5048a import AS5048A
from config import board_pico_drive_03c as pin
from config.board_pico_drive_03c import as5048a_spi


encoder = AS5048A(as5048a_spi,pin.as5048a_cs)

# Example usage
while True:
    response = encoder.read_raw_angle()
    print("Response:", response)
    sleep(1)
