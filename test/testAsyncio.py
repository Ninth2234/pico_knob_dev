import uasyncio

async def blink(led, period_ms):
    while True:
        led.on()
        await uasyncio.sleep_ms(5)
        led.off()
        await uasyncio.sleep_ms(period_ms)

async def task(string,period):
    while True:
        print(string)
        await uasyncio.sleep_ms(period)

async def main(string1,string2):
    uasyncio.create_task(task(string1,100))
    uasyncio.create_task(task(string2,200))
    await uasyncio.sleep_ms(10_000)



# Running on a generic board
from machine import Pin
uasyncio.run(main("hello","world"))