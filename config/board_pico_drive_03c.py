from machine import Pin,SPI,PWM


button          = Pin(10, Pin.IN, Pin.PULL_UP)

## screen gc9a01py
screen_dc       = Pin(12,Pin.OUT)
screen_cs       = Pin(17,Pin.OUT)
screen_reset    = Pin(13,Pin.OUT)
screen_blk      = Pin(16,Pin.OUT)
screen_sck      = Pin(14,Pin.OUT)
screen_mosi     = Pin(15,Pin.OUT)

screen_spi      = SPI(1, baudrate=6_000_000, sck=screen_sck, mosi=screen_mosi)


## magnetic encoder AS5048a
as5048a_miso    = Pin(0)
as5048a_cs      = Pin(1,Pin.OUT)
as5048a_sck     = Pin(2)
as5048a_mosi    = Pin(3)

as5048a_spi     = SPI(0,
                      sck=as5048a_sck,
                      mosi=as5048a_mosi,
                      miso=as5048a_miso,
                      baudrate=6_000_000,
                      polarity=0,
                      phase=1
                      )



## bldc gimbal motor
PWM_FREQ        = 20_000

pwm_u           = PWM(Pin(7, Pin.OUT))
pwm_v           = PWM(Pin(8, Pin.OUT))
pwm_w           = PWM(Pin(9, Pin.OUT))
motor_enable    = Pin(6, Pin.OUT)

motor_enable.off()

pwm_u.freq(PWM_FREQ)
pwm_v.freq(PWM_FREQ)
pwm_w.freq(PWM_FREQ)

pwm_u.duty_u16(0)
pwm_v.duty_u16(0)
pwm_w.duty_u16(0)


