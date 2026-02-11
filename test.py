from machine import Pin
import time

# GPIO0 as output
gpio0 = Pin(0, Pin.OUT)

# 500Hz = 1/500 = 2ms period, toggle every 1ms (1000us)
while True:
    gpio0.value(1)
    time.sleep_us(1000)
    gpio0.value(0)
    time.sleep_us(1000)
