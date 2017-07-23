#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define I2C_FILE "/dev/i2c-1"
using namespace std;

namespace i2c {
static int fd_i2c;
  
int init(void) {
  char *filename = (char*)I2C_FILE;
  if((fd_i2c = open(filename, O_RDWR)) < 0) {
    return -1;
  }
}

int write(char addr, char* data, int length) {
  if (ioctl(fd_i2c, I2C_SLAVE, addr) < 0) {
    return -1;
  } else if (write(fd_i2c, data, length) != length) {
    return -1;
  } else {
    return 0;
  }
}

int read(char addr, char* data, int length) {
  if (ioctl(fd_i2c, I2C_SLAVE, addr) < 0) {
    return -1;
  } else if (read(fd_i2c, buffer, length) != length) {
    return -1;
  } else {
    return 0;
  }
}
};
