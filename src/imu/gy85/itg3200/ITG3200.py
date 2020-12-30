#!/usr/bin/python
# vim: ai:ts=4:sw=4:sts=4:et:fileencoding=utf-8
#
# ITG3200 gyroscope control class
#
# Copyright 2013 Michal Belica <devel@beli.sk>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

import smbus
import time
import math


def int_sw_swap(x):
    """Interpret integer as signed word with bytes swapped"""
    xl = x & 0xff
    xh = x >> 8
    xx = (xl << 8) + xh
    return xx - 0xffff if xx > 0x7fff else xx


class ITG3200(object):
    """ITG3200 digital gyroscope control class.
    Supports data polling at the moment.
    """

    def __init__(self, bus_nr=1, addr=0x68):
        """ Sensor class constructor
        Params:
            bus_nr .. I2C bus number
            addr   .. ITG3200 device address
        """
        self.bus = smbus.SMBus(bus_nr)
        self.addr = addr
        self.offset_x = 0
        self.offset_y = 0
        self.offset_z = 0
        self.default_init()

    def sample_rate(self, lpf, div):
        """Set internal sample rate, low pass filter frequency.
        Sets device parameters DLPF_CFG and SMPLRT_DIV.
        Also sets FS_SEL to 0x03 which is required to initialize the device.
        Params:
            lpf .. (code from the list)
              code   LPF  sample rate
                 0 256Hz  8kHz
                 1 188Hz  1kHz
                 2  98Hz  1kHz
                 3  42Hz  1kHz
                 4  20Hz  1kHz
                 5  10Hz  1kHz
                 6   5Hz  1kHz
            div .. internal sample rate divider (SMPLRT_DIV will be set to div-1)
        """
        if not (lpf >= 0 and lpf <= 0x6):
            raise ValueError("Invalid low pass filter code (0-6).")
        if not (div >= 0 and div <= 0xff):
            raise ValueError("Invalid sample rate divider (0-255).")
        self.bus.write_byte_data(self.addr, 0x15, div - 1)
        self.bus.write_byte_data(self.addr, 0x16, 0x18 | lpf)

    def default_init(self):
        """Initialization with default values:
        8kHz internal sample rate, 256Hz low pass filter, sample rate divider 8.
        """
        self.sample_rate(1, 8)

    def read_data(self, rads=False):
        """Read and return data tuple for x, y and z axis
        as signed 16-bit integers.
        """
        gx = int_sw_swap(self.bus.read_word_data(self.addr, 0x1d))
        gy = int_sw_swap(self.bus.read_word_data(self.addr, 0x1f))
        gz = int_sw_swap(self.bus.read_word_data(self.addr, 0x21))

        if rads:
            to_rads = math.pi / 180
            return to_rads * (gx / 14.375 + self.offset_x), to_rads * (gy / 14.375 + self.offset_y), to_rads * (gz / 14.375 + self.offset_z)
        else:
            return gx / 14.375 + self.offset_x, gy / 14.375 + self.offset_y, gz / 14.375 + self.offset_z

    def calibrate(self):
        """ Auto calibrate the device offset. Put the device so as one axe is parallel to the gravity field (usually, put the device on a flat surface) """
        time.sleep(5)
        sumx, sumy, sumz = 0, 0, 0
        for i in range(10000):
            x, y, z = self.read_data()
            sumx += x
            sumy += y
            sumz += z
        self.offset_x = -sumx / 10000
        self.offset_y = -sumy / 10000
        self.offset_z = -sumz / 10000


if __name__ == '__main__':
    sensor = ITG3200()  # update with your bus number and address

    while True:
        gx, gy, gz = sensor.read_data()
        print(gx, gy, gz)
        time.sleep(1)
