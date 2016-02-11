enum GPIO_PinDir {
  IN = 0, OUT = 1
};

int GPIOExport(int pin);

int GPIOUnexport(int pin);

int GPIODirection(int pin, enum GPIO_PinDir dir);

int GPIORead(int pin);

int GPIOWrite(int pin, int value);
