from config import board_pico_drive_03c as board
from bldc_driver import BLDCDrive
from AS5048a import AS5048A
import time

encoder = AS5048A(board.as5048a_spi,board.as5048a_cs)
motor = BLDCDrive(board.pwm_u,board.pwm_v,board.pwm_w,board.motor_enable)
motor.attach_encoder(encoder.getAngle)

try:
    motor.enable()
    encoder.update()
    encoder.update()    
    encoder.update()

    # while True:
    motor.drive(0.5)
    time.sleep(0.1)
    motor.drive(0)

finally:
    motor.disable()


    
