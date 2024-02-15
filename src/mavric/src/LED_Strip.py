#!/usr/bin/env python3

import rospy, time, os, sys
from std_msgs.msg import String
os.environ["BLINKA_FT232H"] = "1"
import neopixel_spi as neopixel
import board
state = None

def state_cb(data):
    global state
    state = data.data

def listener():
    global state
    NUM_PIXELS = 31
    PIXEL_ORDER = neopixel.GRB
    spi = board.SPI()
    strip = neopixel.NeoPixel_SPI(spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=True)
    red = 0x00FF00
    green = 0xFF0000
    blue = 0x0000FF

    rospy.init_node("LED_Strip")
    state_sub = rospy.Subscriber('State', String, state_cb)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if state == "Idle":
            print("Solid Red, idle autonomous")

        elif state == "TurnTowardWaypoint":
            strip.fill(red)
            time.sleep(1)
            strip.fill(0)
            time.sleep(1)

        elif state == "DriveTowardWaypoint":
            j = 255
            for i in range(51):
                j = j - 5
                strip.fill(hex(j*16**4))
                time.sleep(0.02)
            for i in range(51):
                j = j + 5
                print(hex(j*16**4))
                time.sleep(0.02)

        elif state == "TeleOp":
            strip.fill(blue)

        elif state == "ReachedWaypoint":
            for i in range(5):
                strip.fill(green)
                time.sleep(0.25)
                strip.fill(green)
                time.sleep(0.25)

        else:
            strip.fill(0)
        rate.sleep()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass