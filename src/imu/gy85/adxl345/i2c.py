"""
PYTHON driver for the ADXL-345 (3 axes accelerometer from Analog Device)
This driver use the I2C protocol to communicate (see README)
"""

import smbus
from . import base

class ADXL345(base.ADXL345_Base):

  STD_ADDRESS = 0x1D
  ALT_ADDRESS = 0x53

  def __init__(self, port=1, addr=ALT_ADDRESS):
    """ Initialize the driver
    :param alternate: use the standard or alternate I2C address as selected by pin SDO/ALT_ADDRESS
    :param port: number of I2C bus to use
    """
    super().__init__()
    self.bus = smbus.SMBus(port)
    self.i2caddress = addr

  def get_register(self, address):
    bytes = self.bus.read_i2c_block_data(self.i2caddress, address, 1)
    return bytes[0]

  def get_registers(self, address, count):
    bytes = self.bus.read_i2c_block_data(self.i2caddress, address, count)
    return bytes

  def set_register(self, address, value):
    self.bus.write_byte_data(self.i2caddress, address, value)

