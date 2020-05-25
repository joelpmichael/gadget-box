#!/usr/bin/env python3

import smbus
import RPi.GPIO as GPIO

import time
import signal
import sys

i2c = smbus.SMBus(1)

MCP23017_0_ADDR = 0x20
MCP23017_1_ADDR = 0x21

MCP23017_RST_PIN = 17
MCP23017_0_INT_PIN = 27
MCP23017_1_INT_PIN = 22

MCP23X17_REG_IOCONA = 0x05
MCP23X17_REG_IOCONB = MCP23X17_REG_IOCONA + 0x10
MCP23X17_REG_IOCONA_ALTA = 0x0A # this is the same address as MCP23X17_REG_OLATA in BANK mode
MCP23X17_REG_IOCONB_ALTB = 0x0B # only guaranteed known address in either BANK mode

MCP23X17_REG_IODIRA = 0x00
MCP23X17_REG_IODIRB = MCP23X17_REG_IODIRA + 0x10

MCP23X17_REG_IPOLA = 0x01
MCP23X17_REG_IPOLB = MCP23X17_REG_IPOLA + 0x10

MCP23X17_REG_GPINTENA = 0x02
MCP23X17_REG_GPINTENB = MCP23X17_REG_GPINTENA + 0x10

MCP23X17_REG_DEFVALA = 0x03
MCP23X17_REG_DEFVALB = MCP23X17_REG_DEFVALA + 0x10

MCP23X17_REG_INTCONA = 0x04
MCP23X17_REG_INTCONB = MCP23X17_REG_INTCONA + 0x10

MCP23X17_REG_GPPUA = 0x06
MCP23X17_REG_GPPUB = MCP23X17_REG_GPPUA + 0x10

MCP23X17_REG_INTFA = 0x07
MCP23X17_REG_INTFB = MCP23X17_REG_INTFA + 0x10

MCP23X17_REG_INTCAPA = 0x08
MCP23X17_REG_INTCAPB = MCP23X17_REG_INTCAPA + 0x10

MCP23X17_REG_GPIOA = 0x09
MCP23X17_REG_GPIOB = MCP23X17_REG_GPIOA + 0x10

MCP23X17_REG_OLATA = 0x0A
MCP23X17_REG_OLATB = MCP23X17_REG_OLATA + 0x10

MCP23X17_CFG_IOCON = 0b11101000 # set desired chip configuration - ss3.5.6
# BANK = 1 (port registers in blocks)
# MIRROR = 1 (INT pins OR'ed together)
# SEQOP = 1 (sequential access disabled)
# DISSLW = 0 (clock slew enabled)
# HAEN = 1 (hardware address enabled (always enabled on MCP23017))
# ODR = 0 (active driver output)
# INTPOL = 0 (active-low interrupt)
# (reserved) = 0

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def MCP23X17_INT_cb(channel):
    # debounce after detecting interrupt
    # debounce code ported from Kenneth Kuhn's original C code at http://www.kennethkuhn.com/electronics/debounce.c
    print("MCP23X17_INT_cb called")
    gpio_state = i2c.read_byte_data(MCP23017_0_ADDR, MCP23X17_REG_INTCAPA)
    print("INTCAPA State: {0:08b}".format(gpio_state))
    gpio_state = i2c.read_byte_data(MCP23017_0_ADDR, MCP23X17_REG_GPIOA)
    print("  GPIOA State: {0:08b}".format(gpio_state))
    gpio_state = i2c.read_byte_data(MCP23017_0_ADDR, MCP23X17_REG_INTCAPB)
    print("INTCAPB State: {0:08b}".format(gpio_state))
    gpio_state = i2c.read_byte_data(MCP23017_0_ADDR, MCP23X17_REG_GPIOB)
    print("  GPIOB State: {0:08b}".format(gpio_state))

if __name__ == '__main__':
    # setup RPi GPIO pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MCP23017_RST_PIN, GPIO.OUT, GPIO.PUD_OFF, initial=GPIO.HIGH)
    GPIO.setup(MCP23017_0_INT_PIN, GPIO.IN)
    GPIO.setup(MCP23017_1_INT_PIN, GPIO.IN)

    # setup MCP23X17
    # tie down RESET for a bit
    GPIO.output(MCP23017_RST_PIN, GPIO.LOW)
    time.sleep(0.001)
    GPIO.output(MCP23017_RST_PIN, GPIO.HIGH)
    time.sleep(0.001)

    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_IOCONB_ALTB, MCP23X17_CFG_IOCON) # on PoR, bank = 0 so try ALTB address first
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_IOCONB, MCP23X17_CFG_IOCON) # if bank = 1 (i.e. no PoR) this address will work too

    # set all IO pins as input - ss3.5.1
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_IODIRA, 0b11111111)
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_IODIRB, 0b11111111)

    # set all IO pins to non-inverting polarity - ss3.5.2
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_IPOLA, 0b00000000)
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_IPOLB, 0b00000000)

    # set default compare value to trigger interrupt when INTCON is set to 1 for pin - ss3.5.4
    # (this doesn't use DEFVAL)
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_DEFVALA, 0b00000000)
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_DEFVALB, 0b00000000)

    # set INTCON to trigger interrupt when value changes from DEFVAL - ss3.5.5
    # (this doesn't use INTCON)
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_INTCONA, 0b00000000)
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_INTCONB, 0b00000000)

    # trigger interrupt on change on all ports
    # INTCON = 0, so interrupt will occur when value changes from last value
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_GPINTENA, 0b11111111)
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_GPINTENB, 0b11111111)

    # set GPIO pull-up resistors
    # all disabled, using external pull-up resistors
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_GPPUA, 0b00000000)
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_GPPUB, 0b00000000)

    # set GPIO output latches
    # all low, all pins are set to input
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_OLATA, 0b00000000)
    i2c.write_byte_data(MCP23017_0_ADDR, MCP23X17_REG_OLATB, 0b00000000)

    # MCP23X17 setup complete, read GPIO registers to clear interrupt
    i2c.read_byte_data(MCP23017_0_ADDR, MCP23X17_REG_GPIOA)
    i2c.read_byte_data(MCP23017_0_ADDR, MCP23X17_REG_GPIOB)

    # setup GPIO interrupt inputs
    GPIO.add_event_detect(MCP23017_0_INT_PIN, GPIO.FALLING, callback=MCP23X17_INT_cb)
    GPIO.add_event_detect(MCP23017_1_INT_PIN, GPIO.FALLING, callback=MCP23X17_INT_cb)

    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()
