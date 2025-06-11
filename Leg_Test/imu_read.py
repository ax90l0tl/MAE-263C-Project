import serial
import time
from MSCL import mscl
import gc
import struct
from leg_manager import *


port = 'COM20'
baud = 115200
delay = 0.005
PACKET_SIZE = 57
SYNC_BYTES = b'\x75\x65\x80'

leg = legManager(bear_port='COM3', sensor_port=port)
while True:
    print(leg.y_acc, leg.gpio_state)
    time.sleep(0.01)
leg.stop_imu()


# def init_imu(COM_PORT, GPIO_PIN=1):
#     try:
#         #create a Serial Connection with the specified COM Port, default baud rate of 921600
#         connection = mscl.Connection.Serial(COM_PORT)
#         #create an InertialNode with the connection
#         node = mscl.InertialNode(connection)
#     except mscl.Error as e:
#         print("Error:", e)
#         exit(1)
#     # To use a GPIO pin, we first need to configure it
#     try:
#         new_config = mscl.GpioConfiguration()
#         new_config.pin = GPIO_PIN  # GPIO pin number
#         new_config.feature = new_config.GPIO_FEATURE  # Set pin as GPIO
#         new_config.behavior = new_config.GPIO_INPUT  # Set GPIO pin as input
#         # node.setGpioConfig(new_config)  # Apply the configuration
#         del node
#     except mscl.Error as e:
#         print("Error:", e)
#         exit(1)
#     connection.disconnect()
#     return

# def sync_and_read_packet(ser):
#     while True:
#         byte = ser.read(1)
#         if byte == SYNC_BYTES[0:1]:
#             next_byte = ser.read(1)
#             if next_byte == SYNC_BYTES[1:2]:
#                 next_byte = ser.read(1)
#                 if next_byte == SYNC_BYTES[2:3]:
#                     # We've found the sync sequence
#                     rest = ser.read(PACKET_SIZE - 3)
#                     if len(rest) == PACKET_SIZE - 3:
#                         return SYNC_BYTES + rest  # Return full 64-byte packet  

# init_imu(port)
# time.sleep(0.5)
# imu = serial.Serial(port=port, baudrate=baud, timeout=1)

# while True:
#     packet = sync_and_read_packet(imu)
#     print("Received packet:", ' '.join(f'{b:02X}' for b in packet))
#     # y_acc_raw = packet[24:28]
#     # y_acc = struct.unpack('>f', bytearray(y_acc_raw))
#     # print(y_acc[0])
#     time.sleep(delay)
