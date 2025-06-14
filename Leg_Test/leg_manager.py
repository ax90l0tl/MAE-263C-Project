from pybear import Manager
from pybear.CONTROL_TABLE import *
import mscl
import numpy as np
import time
import serial
import threading
import struct

'''
Prior to running this wrapper, ensure that you have:
    - 2 BEAR motors with distinct IDs connected to the specified COM port
    - The IMU with foot sensor connected to the specified COM port
'''

class legManager:
    def __init__(self, bear_port='COM3', sensor_port='COM20', baudrate=8000000, gpio_pin=1):
        self.bear_port = bear_port
        self.sensor_port = sensor_port
        self.baudrate = baudrate
        self.gpio_pin = gpio_pin
        self.bear_kt = 1.16  # Nm/A, from BEAR SDK. Koala: 0.35, Koala MB: 1.16.
        self.iq_max = 20
        self.PACKET_SIZE = 57
        self.SYNC_BYTES = b'\x75\x65\x80'

        self.bear = Manager.BEAR(port=self.bear_port, baudrate=self.baudrate)
        self.bear_ids = self._search_bear() 
        self._configure_bear_motors()
        self.imu = self._init_imu()  # GPIO pin 1 for foot sensor

        self.gpio_state = 0
        self.y_acc = 0

        # self.freq = 300
        # self._running = True
        # self._thread = threading.Thread(target=self._imu_polling, daemon=True)
        # self._thread.start()

    def _init_imu(self):
        try:
            #create a Serial Connection with the specified COM Port, default baud rate of 921600
            connection = mscl.Connection.Serial(self.sensor_port)
            #create an InertialNode with the connection
            node = mscl.InertialNode(connection)
        except mscl.Error as e:
            print("Error:", e)
            exit(1)
        # To use a GPIO pin, we first need to configure it
        try:
            new_config = mscl.GpioConfiguration()
            new_config.pin = self.gpio_pin  # GPIO pin number
            new_config.feature = new_config.GPIO_FEATURE  # Set pin as GPIO
            new_config.behavior = new_config.GPIO_INPUT  # Set GPIO pin as input
            node.setGpioConfig(new_config)  # Apply the configuration
            # del node
        except mscl.Error as e:
            print("Error:", e)
            exit(1)
        # connection.disconnect()
        # time.sleep(0.5)
        # ser = serial.Serial(self.sensor_port, 115200)
        # return ser
        return node
    
    def _sync_and_read_packet(self):
        while True:
            byte = self.imu.read(1)
            if byte == self.SYNC_BYTES[0:1]:
                next_byte = self.imu.read(1)
                if next_byte == self.SYNC_BYTES[1:2]:
                    next_byte = self.imu.read(1)
                    if next_byte == self.SYNC_BYTES[2:3]:
                        # We've found the sync sequence
                        rest = self.imu.read(self.PACKET_SIZE - 3)
                        if len(rest) == self.PACKET_SIZE - 3:
                            return self.SYNC_BYTES + rest  # Return full 64-byte packet
    
    def _imu_polling(self):
        period = 1.0 / self.freq
        while self._running:
            packet = self._sync_and_read_packet()
            # print("Received packet:", ' '.join(f'{b:02X}' for b in packet))
            y_acc_raw = packet[24:28]
            y_acc = struct.unpack('>f', bytearray(y_acc_raw))
            self.y_acc = y_acc[0]
            self.gpio_state = int(packet[54])
            time.sleep(period)

    def stop_imu(self):
        """Gracefully stop the background thread."""
        self._running = False
        self._thread.join()

    def flush_imu_buffer(self):
        self.imu.reset_input_buffer()
        return
    
    def get_foot_sensor_state(self):
        return self.imu.getGpioState(self.gpio_pin)
        # return self.gpio_state
    
    def get_imu_data(self):
        return self.y_acc

    def _search_bear(self):
        searched_list = []
        for i in range(0, 10):
            data = self.bear.ping(i)
            if data[0] is not None:
                print(f"Found BEAR with ID {i}.")
                searched_list.append(i)
        return searched_list
    
    def _configure_bear_motors(self, mode = 0, kp = 0.358, ki = 0.045, kd = 0):
        for id in self.bear_ids:
            self.bear.set_torque_enable((id, 0))
            self.bear.set_mode((id, 0))
            self.bear.set_p_gain_id((id, kp))
            self.bear.set_p_gain_iq((id, kp))
            self.bear.set_i_gain_id((id, ki))
            self.bear.set_i_gain_iq((id, ki))
            self.bear.set_d_gain_id((id, kd))
            self.bear.set_d_gain_iq((id, kd))
        return
    
    def enable_torque(self, state = 0):
        for id in self.bear_ids:
            self.bear.set_torque_enable((id, int(state)))
        return

    def get_feedback(self):
        """
        |  Read multiple status registers from a single motor in one packet.
        |  Multiple target motors can be visited but goes through them one-by-one
        |  e.g.: get_status((ID, reg1, reg2), (ID, reg1, reg2, reg3))
        """
        results = self.bear.get_status((1, 'present_iq', 'present_velocity', 'present_position'),
                                       (2, 'present_iq', 'present_velocity', 'present_position'))
        # print(f"Feedback from BEAR motors: {results}")
        iq1 = results[0][0][0]
        iq2 = results[1][0][0]
        q1d_rad_s = results[0][0][1]
        q2d_rad_s = results[1][0][1]
        q1_rad = results[0][0][2]
        q2_rad = results[1][0][2]
        return np.array([iq1, iq2]), np.array([q1_rad, q2_rad]), np.array([q1d_rad_s, q2d_rad_s])
    
    def iq_to_torque(self, iq):
        return self.bear_kt * iq
    
    def torque2iq(self, u):
        u1, u2 = u
        iq1 = u1 / self.bear_kt
        iq2 = u2 / self.bear_kt
        iq1 = np.clip(iq1, -self.iq_max, self.iq_max)
        iq2 = np.clip(iq2, -self.iq_max, self.iq_max)
        return [iq1, iq2]
    
    def iq2torque(self, iq):
        iq1, iq2 = iq
        u1 = iq1 * self.bear_kt
        u2 = iq2 * self.bear_kt
        return [u1, u2]
        
    def move_motors(self, goal_u):
        goal_iq = self.torque2iq(goal_u)
        self.bear.set_goal_iq((self.bear_ids[0], goal_iq[0]), 
                              (self.bear_ids[1], goal_iq[1]))
        return
