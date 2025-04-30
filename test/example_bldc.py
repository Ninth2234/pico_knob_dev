from machine import Pin, PWM,SPI
from time import sleep
from AS5048a import AS5048A
import math


pwm_freq = 20_000
duty = 10000  # ~50% duty cycle (range 0–65535)

# Initialize 3 PWM phases
u = PWM(Pin(7, Pin.OUT))
v = PWM(Pin(8, Pin.OUT))
w = PWM(Pin(9, Pin.OUT))

ena = Pin(6, Pin.OUT)
ena.off()

for phase in [u, v, w]:
    phase.freq(pwm_freq)  
    phase.duty_u16(0)



# Define SPI pins
miso = Pin(0)
cs = Pin(1, Pin.OUT)  # Chip Select as output
clk = Pin(2)
mosi = Pin(3)

# Initialize SPI (e.g., SPI(0) for SPI0 or SPI(1) for SPI1)
spi = SPI(0, 
        #   baudrate=1000000,       # 1 MHz, adjust as needed
          polarity=0,             # Clock polarity
          phase=1,                # Clock phase
        #   bits=8,                 # 8 bits per transfer
        #   firstbit=SPI.MSB,       # MSB first
          sck=clk,
          mosi=mosi,
          miso=miso)

encoder = AS5048A(spi,cs)


# Constants (make sure these are defined elsewhere appropriately)
PI = math.pi
PI_2 = math.pi / 2
PI_3 = math.pi / 3
DEG_2_RAD = math.pi/180
SQRT3 = math.sqrt(3)

# Dummy values (replace with your actual config)
PWM_MAX_DUTYU16 = 65535
zero_electric_angle = 0

# Replace with actual PWM setup
def setPwm(Ua, Ub, Uc):
    # Convert voltages to PWM duty cycle (0.0 to 1.0)
    # and set it to your motor control pins
    duty_a = Ua*PWM_MAX_DUTYU16
    duty_b = Ub*PWM_MAX_DUTYU16
    duty_c = Uc*PWM_MAX_DUTYU16

    u.duty_u16(int(duty_a))
    v.duty_u16(int(duty_b))
    w.duty_u16(int(duty_c))
    

# Normalize angle between 0 and 2*pi
def normalizeAngle(angle):
    return angle % (2 * PI)

def setPhaseVoltage(Uq, angle_el):
    """_summary_

    Args:
        Uq (float): input effort [-1 1]
        angle_el (_type_): _description_
    """
    if Uq < 0:
        angle_el += PI
    Uq = abs(Uq)

    angle_el = normalizeAngle(angle_el + zero_electric_angle + PI_2)
    sector = int(angle_el / PI_3) + 1

    T1 = SQRT3 * math.sin(sector * PI_3 - angle_el) * Uq
    T2 = SQRT3 * math.sin(angle_el - (sector - 1.0) * PI_3) * Uq
    T0 = 1 - T1 - T2

    if sector == 1:
        Ta = T1 + T2 + T0 / 2
        Tb = T2 + T0 / 2
        Tc = T0 / 2
    elif sector == 2:
        Ta = T1 + T0 / 2
        Tb = T1 + T2 + T0 / 2
        Tc = T0 / 2
    elif sector == 3:
        Ta = T0 / 2
        Tb = T1 + T2 + T0 / 2
        Tc = T2 + T0 / 2
    elif sector == 4:
        Ta = T0 / 2
        Tb = T1 + T0 / 2
        Tc = T1 + T2 + T0 / 2
    elif sector == 5:
        Ta = T2 + T0 / 2
        Tb = T0 / 2
        Tc = T1 + T2 + T0 / 2
    elif sector == 6:
        Ta = T1 + T2 + T0 / 2
        Tb = T0 / 2
        Tc = T1 + T0 / 2
    else:
        Ta = Tb = Tc = 0

    Ua = Ta
    Ub = Tb
    Uc = Tc
    print(angle_el/DEG_2_RAD)
    setPwm(Ua, Ub, Uc)




def cal_el_angle(mech_angle):
    zero_angle = 41
    n_pole = 7
    return (zero_angle-mech_angle)*n_pole
    return (mech_angle/(340.6641-289.314)*(0-360))

try:
    ena.high()
    while True:
        encoder.update()
        mech_angle = encoder.getAngle()
        el_angle = cal_el_angle(mech_angle)*DEG_2_RAD
        setPhaseVoltage(0.1,el_angle)
        print(mech_angle,el_angle)
    
finally:
    ena.low()
# while True:
#     encoder.update()
#     print(encoder.getAngle())
#     sleep(0.1)
# try:
#     ena.high()
#     while True:
#         for i in range(360):
#             setPhaseVoltage(0.2,i*DEG_2_RAD)

#             encoder.update()
#             mech_angle = encoder.getAngle()
#             el_angle = cal_el_angle(mech_angle)
#             print(mech_angle,i)
#             sleep(0.01)


# finally:
#     ena.low()