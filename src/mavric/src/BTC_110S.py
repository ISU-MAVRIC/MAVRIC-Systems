#!/usr/bin/env python
import serial

# Necessarry Commands:
#  Q - reset spectrometer.
#  a - set ascii mode (data)
#  b - set binary mode (data)
#  K# - set baud rate (initially 9600) with the following mappings:
#         7 = 600 baud
#         6 = 1200 baud
#         5 = 2400 baud
#         4 = 4800 baud
#         3 = 9600 baud
#         2 = 19200 baud
#         1 = 38400 baud
#         0 = 115200 baud
#
#  S - perform scan:

#  Data format:
#    The device echos the command, then gives an "ACK\r\n" response
#    After that, for a scan, the bytes are data.
#    In binary mode, the first pixel (of 2048) is transmitted as 3 bytes, starting with 0x80, and then the MSB, then the LSB.
#    After that, if the byte is 0x80, then the next 2 bytes are the MSB, LSB of the next pixel.
#                otherwise, the next 1 byte is the signed 8-bit representation of the difference between the previous pixel and this one.
#                  e.g. If the previous pixel was 1000, and the next byte is 0xFF, then this pixel is 999.
#                  e.g. If the previous pixel was 1000, and the next byte is 0x08, then this pixel is 1008.
