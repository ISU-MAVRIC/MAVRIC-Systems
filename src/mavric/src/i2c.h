#ifndef _I2C_H_
#define _I2C_H_

namespace i2c {
  int init(void);
  int write(char addr, char* data, int length);
  int read(char addr, char* data, int length);
};

#endif // _I2C_H_
