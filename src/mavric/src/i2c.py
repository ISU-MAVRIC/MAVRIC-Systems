#!/usr/bin/env python

import io
import fcntl

I2C_SLAVE=0x0703
    
class I2C:

    def __init__(self, device, bus):
        self._addr = device
        self._bus = bus
        self._isOpen = False

    def open(self):
        if (self._isOpen):
            return
        
        self.fw = io.open("/dev/i2c-"+str(self._bus), "wb", buffering=0)
        self.fr = io.open("/dev/i2c-"+str(self._bus), "rb", buffering=0)
        # set device address
        fcntl.ioctl(self.fr, I2C_SLAVE, self._addr)
        fcntl.ioctl(self.fw, I2C_SLAVE, self._addr)
       
    def close(self):
        if (not self._isOpen):
            return
        self.fw.close()
        self.fr.close()
        self.fw = None;
        self.fr = None;

    def write(self, bytes):
        wasOpen = self._isOpen
        if (not wasOpen):
            self.open()
            print(bytes)
        self.fw.write(bytes)
        if (not wasOpen):
            self.close()

    def read(self, bytes):
        wasOpen = self._isOpen
        if (not wasOpen):
            self.open()
        string = self.fr.read(bytes)
        if (not wasOpen):
            self.close()
        bytes = bytearray(string)
        return bytes
            

if __name__ == "__main__":
    import i2c
    dev = i2c.I2C(0x49, 1)
    dev.open()
    dev.close()
    data = dev.read(10)

    print(data)
    print(''.join('{:02x}'.format(x) for x in data))
