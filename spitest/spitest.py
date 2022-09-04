#!/usr/bin/env python3
import pyftdi.spi
import time

if __name__ == '__main__':
    spi = pyftdi.spi.SpiController()
    spi.configure("ftdi://::/1")
    port = spi.get_port(cs=0, mode=0, freq=10e6)
    gpio = spi.get_gpio()
    gpio.set_direction(0x20, 0x20)
    gpio.write(0x20)
    while True:
        port.write(bytes([1,2,3,4]))
        time.sleep(1)
    # gpio.write(0x00)
