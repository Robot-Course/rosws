#!/usr/bin/env python
# vim: set fileencoding=UTF-8 :

# HMC5888L Magnetometer (Digital Compass) wrapper class
# Based on https://bitbucket.org/thinkbowl/i2clibraries/src/14683feb0f96,
# but uses smbus rather than quick2wire and sets some different init
# params.

import smbus
import math
import time
import sys


class HMC5883L:
    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=1, address=0x1E, gauss=1.3, declination=(0, 0)):
        self.bus = smbus.SMBus(port)
        self.address = address

        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180

        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.address, 0x00, 0x70)  # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x01, reg << 5)  # Scale
        self.bus.write_byte_data(self.address, 0x02, 0x00)  # Continuous measurement

    def declination(self):
        return (self.__declDegrees, self.__declMinutes)

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1 << len)
        return val

    def __convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset + 1], 16)
        if val == -4096: return None
        return round(val * self.__scale, 4)

    def read_data(self):
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        # print map(hex, data)
        x = self.__convert(data, 3)
        y = self.__convert(data, 7)
        z = self.__convert(data, 5)
        return (x, y, z)

    def heading(self, pitch, roll):
        (x, y, z) = self.read_data()
        x *= 9.8
        y *= 9.8
        z *= 9.8
        # headingRad = math.atan2(y, x)
        mag_x = x * math.cos(pitch) + y * math.sin(roll) * math.sin(pitch) + z * math.cos(roll) * math.sin(pitch)
        mag_y = y * math.cos(roll) - z * math.sin(roll)
        headingRad = math.atan2(mag_y, mag_x)
        headingRad += self.__declination

        # Correct for reversed heading
        if headingRad < 0:
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        elif headingRad > 2 * math.pi:
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = headingRad * 180 / math.pi
        return headingDeg

    def degrees(self, headingDeg):
        degrees = math.floor(headingDeg[0])
        minutes = round((headingDeg[1] - degrees) * 60)
        return (degrees, minutes)

    def __str__(self):
        (x, y, z) = self.read_data()
        return "Axis X: " + str(x) + "\n" + \
               "Axis Y: " + str(y) + "\n" + \
               "Axis Z: " + str(z) + "\n" + \
               "Declination: " + str(self.degrees(self.declination())) + "\n" + \
               "Heading: " + str(self.heading()) + "\n"


if __name__ == "__main__":
    # http://magnetic-declination.com/Great%20Britain%20(UK)/Harrogate#
    compass = HMC5883L(gauss=4.7, declination=(-2, 5))
    while True:
        print(compass)
        time.sleep(5)
