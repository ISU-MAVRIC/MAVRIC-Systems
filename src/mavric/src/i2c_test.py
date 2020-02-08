from i2c import I2C

bus = I2C(0x40, 1)

bus.open()
bus.write(b"\2\4")
bus.close()
