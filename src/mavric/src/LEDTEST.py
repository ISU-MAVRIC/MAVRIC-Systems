import time
import os
import sys
os.environ["BLINKA_FT232H"] = "1"
import board 
import neopixel_spi as neopixel

NUM_PIXELS = 31
PIXEL_ORDER = neopixel.GRB
COLORS = (0xFF0000, 0x00FF00, 0x0000FF)
DELAY = 0.5

spi = board.SPI()

pixels = neopixel.NeoPixel_SPI(spi,
                               NUM_PIXELS,
                               pixel_order=PIXEL_ORDER,
                               auto_write=False)

def main():
    while True:
        for color in COLORS:
            for i in range(NUM_PIXELS): 
                pixels[i] = color 
                pixels.show()
                time.sleep(DELAY)
                pixels.fill(0)
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        try:
            pixels.fill(0)
            pixels.show() 
            sys.exit()
        except SystemExit:
            os._exit(130)