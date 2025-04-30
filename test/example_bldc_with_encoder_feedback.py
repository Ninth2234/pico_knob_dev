from config import board_pico_drive_03c as board
from bldc_driver import BLDCDrive
from AS5048a import AS5048A


encoder = AS5048A(board.as5048a_spi,board.as5048a_cs)
motor = BLDCDrive(board.pwm_u,board.pwm_v,board.pwm_w,board.motor_enable)
motor.attach_encoder(encoder.getAngle)

try:
    motor.enable()
    while True:
        encoder.update()
        motor.drive(0.1)

finally:
    motor.disable()


    
