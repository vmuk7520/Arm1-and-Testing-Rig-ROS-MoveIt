import time
import random
import serial
import argparse
import signal
import sys
import traceback
from enum import Enum, auto
from typing import Dict, Union

import rospy
from sensor_msgs.msg import JointState
import numpy as np


class CANUSB_SPEED(Enum):
    SPEED_1000000 = 0x01
    SPEED_800000 = 0x02
    SPEED_500000 = 0x03
    SPEED_400000 = 0x04
    SPEED_250000 = 0x05
    SPEED_200000 = 0x06
    SPEED_125000 = 0x07
    SPEED_100000 = 0x08
    SPEED_50000 = 0x09
    SPEED_20000 = 0x0a
    SPEED_10000 = 0x0b
    SPEED_5000 = 0x0c


class CANUSB_MODE(Enum):
    NORMAL = 0x00
    LOOPBACK = 0x01
    SILENT = 0x02
    LOOPBACK_SILENT = 0x03


class CANUSB_FRAME(Enum):
    STANDARD = 0x01
    EXTENDED = 0x02


class CANUSB_PAYLOAD_MODE(Enum):
    INJECT_PAYLOAD_MODE_RANDOM = 0
    INJECT_PAYLOAD_MODE_INCREMENTAL = 1
    INJECT_PAYLOAD_MODE_FIXED = 2


class SerialPortError(Exception):
    pass


class UsbCanAdapter:
    """A class to interact with a USB CAN adapter."""

    CANUSB_INJECT_SLEEP_GAP_DEFAULT = 200  # ms
    CANUSB_TTY_BAUD_RATE_DEFAULT = 2000000
    DATA_START_INDEX = 4
    DATA_TYPE_SLICE = slice(1, 2)
    FRAME_ID_SLICE = slice(2, 4)

    def __init__(self):
        self.device_port = None
        self.speed = CANUSB_SPEED.SPEED_1000000
        self.baudrate = self.CANUSB_TTY_BAUD_RATE_DEFAULT
        self.terminate_after = 0
        self.program_running = True
        self.inject_payload_mode = CANUSB_PAYLOAD_MODE.INJECT_PAYLOAD_MODE_FIXED
        self.inject_sleep_gap = self.CANUSB_INJECT_SLEEP_GAP_DEFAULT
        self.print_traffic = False
        self.frame = bytearray()
        self.serial_device = None
        self.data_dict = {}

    @staticmethod
    def canusb_int_to_speed(speed: int) -> CANUSB_SPEED:
        speed_dict = {
            1000000: CANUSB_SPEED.SPEED_1000000,
            800000: CANUSB_SPEED.SPEED_800000,
            500000: CANUSB_SPEED.SPEED_500000,
            400000: CANUSB_SPEED.SPEED_400000,
            250000: CANUSB_SPEED.SPEED_250000,
            200000: CANUSB_SPEED.SPEED_200000,
            125000: CANUSB_SPEED.SPEED_125000,
            100000: CANUSB_SPEED.SPEED_100000,
            50000: CANUSB_SPEED.SPEED_50000,
            20000: CANUSB_SPEED.SPEED_20000,
            10000: CANUSB_SPEED.SPEED_10000,
            5000: CANUSB_SPEED.SPEED_5000
        }
        return speed_dict.get(speed, 0)

    @staticmethod
    def generate_checksum(data: bytearray) -> int:
        """Generates a checksum from the given data bytearray."""
        checksum = sum(data)
        return checksum & 0xff

    def frame_send(self, frame: bytearray) -> int:
        """Sends a frame to the USB-CAN-ADAPTER device."""
        if not self.serial_device.is_open:
            raise SerialPortError("Serial port is not open.")
        frame_len = len(frame)
        try:
            result = self.serial_device.write(bytes(frame))
        except serial.SerialException as e:
            raise SerialPortError(f"write() failed: {e}")
        return frame_len

    def frame_receive(self, frame_len_max: int = 20) -> int:
        """Receives a frame from the USB-CAN-ADAPTER device."""
        if not self.serial_device.is_open:
            print("Error: Serial port is not open.")
            return -1

        self.frame = bytearray()
        frame_len = 0
        started = False

        if self.print_traffic:
            print("<<< ", end="")

        while self.program_running and frame_len < frame_len_max:
            try:
                byte = self.serial_device.read(1)
            except serial.SerialException as e:
                print(f"Error reading from serial port: {e}")
                return -1

            if self.print_traffic:
                print(f"{byte[0]:02x} ", end="")

            if byte[0] == 0x55 and started:
                self.frame.append(byte[0])
                frame_len += 1
                break

            if byte[0] == 0xaa:
                started = True

            if started:
                self.frame.append(byte[0])
                frame_len += 1

            if frame_len >= 32:
                break

        if self.print_traffic:
            print('')
        return frame_len

    def command_settings(self, speed: int = 0, mode: CANUSB_MODE = CANUSB_MODE.NORMAL,
                         frame: CANUSB_FRAME = CANUSB_FRAME.STANDARD) -> int:

        if speed != 0:
            self.speed = self.canusb_int_to_speed(speed)
        """
        Configures the CAN to serial adapter settings.

        This function sets up the communication protocol for the CAN to serial adapter. The communication can be in either a fixed 20-byte communication protocol, or a variable length communication protocol based on the settings provided.

        Parameters:
        speed (int, optional): The speed of the CAN bus in bps. It is converted to an enum value based on predefined speed settings. Default is 0 which corresponds to a pre-configured speed setting.
        mode (CANUSB_MODE, optional): The mode in which the CAN to serial adapter operates. The modes can be NORMAL, LOOPBACK, SILENT, or LOOPBACK_SILENT. Default is NORMAL.
        frame (CANUSB_FRAME, optional): The frame format, which can be STANDARD or EXTENDED. Default is STANDARD.

        Variable Length Communication Protocol:
        In this protocol, the packet structure is as follows:
        - Packet Header: 0xaa
        - Type: 0xc0, with various bits indicating frame type, frame format and data length.
        - Frame ID: 2 bytes for standard frame and 4 bytes for extended frame.
        - Frame Data: 0 to 8 data bytes.
        - End Code: 0x55

        Fixed 20-byte Communication Protocol:
        In this protocol, the packet structure is as follows:
        - Packet Header: 0xaa, 0x55
        - Type: 0x01 for Data
        - Frame Type: 0x01 for Standard frame, 0x02 for Extended frame.
        - Frame Format: 0x01 for Data frame, 0x02 for Remote frame.
        - Frame ID Data: 4 bytes, high bytes at the front, low bytes at the back.
        - Frame Data Length: The data length of the CAN bus that is sent or accepted.
        - Frame Data: Up to 8 data bytes.
        - Reserve: 0x00
        - Check Code: Checksum from frame type to error code, accumulating and taking the low 8 bits.

        Returns:
        int: Returns 0 on success, and -1 on failure in sending the command frame to the adapter.
        """
        cmd_frame = bytearray()

        cmd_frame.append(0xaa)
        cmd_frame.append(0x55)
        cmd_frame.append(0x12)

        cmd_frame.append(self.speed.value)
        cmd_frame.append(frame.value)
        cmd_frame.extend([0] * 8)  # Fill with zeros for Filter ID and Mask ID (not handled)
        cmd_frame.append(mode.value)
        cmd_frame.extend([0x01, 0, 0, 0, 0])
        cmd_frame.append(self.generate_checksum(cmd_frame[2:19]))

        if self.frame_send(cmd_frame) < 0:
            return -1

        return 0

    def extract_data(self, frame: bytearray) -> Dict[str, Union[bytearray, str]]:
        """
        Extracts the frame ID, PGN, node, and data bytes from a CAN frame.

        Args:
            frame (bytearray): A bytearray containing the CAN frame data.

        Returns:
            dict: A dictionary containing the following keys:
                * frame_id: The frame ID (4 bytes) as a string.
                * pgn: The parameter group number (2 bytes) as a string.
                * node: The node ID (1 byte) as a string.
                * data: A bytearray containing the data bytes.
        """
        try:
            frame_id_bytes = frame[self.FRAME_ID_SLICE][::-1]
            frame_id = frame_id_bytes.hex()

            data_type_byte = frame[self.DATA_TYPE_SLICE][::-1]
            data_length = int(data_type_byte.hex()[1])

            # Use slicing to extract data bytes and reverse the order of every adjacent pair of bytes
            data_bytes = []
            for i in range(self.DATA_START_INDEX, (self.DATA_START_INDEX+data_length), 2):
                if len(frame) == 13:
                    lsB = frame[i]
                    msB = frame[i+1]
                    conversion = np.pi*2/(2**16)
                    angle = ((msB<<8)|lsB)*conversion
                    data_bytes.append(angle)

            pi_two = np.pi * 2
            converted_radians = []
            for angle in data_bytes:
                # Normalize the angle to between 0 and 2*pi
                normalized_angle = angle % pi_two
                # Convert to -pi to pi range
                if normalized_angle > np.pi:
                    converted_angle = normalized_angle - pi_two
                else:
                    converted_angle = normalized_angle
                converted_radians.append(converted_angle)

        except IndexError as e:
            error_message = (f"Error in CAN data frame\nException: {e}\nTraceback:\n{traceback.format_exc()}")
            print(error_message)

        return {"frame_id": frame_id, "data_length:": data_length//2, "data": converted_radians}

    def dump_data_frames(self, print_flag: bool) -> int:
        """Dumps data frames to the console or log."""

        rospy.init_node('joint_state_publisher')

        while self.program_running:
            frame_len = self.frame_receive(20)

            if not self.program_running:
                break

            if frame_len == -1 or frame_len != 13:
                print("Frame receive error!")
            else:
                self.data_dict = self.extract_data(self.frame)

                  # Define joint names and desired positions
                joint_names = ['shoulder', 'elbow_1', 'elbow_2', 'wrist']

                # Create a JointState message
                joint_state = JointState()
                joint_state.header.stamp = rospy.Time.now()  # Set timestamp

                joint_state.name = joint_names
                joint_state.position = self.data_dict["data"]

                # Publish the message
                pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
                pub.publish(joint_state)

            if print_flag:
                try:
                    print(f"{self.data_dict}")
                except KeyError:
                    pass
        return 0

    def adapter_init(self, device_port: str = None, baudrate: int = None) -> serial.Serial:
        """Initializes the adapter with the specified device port and baudrate."""
        if device_port != None:
            self.device_port = device_port
        if baudrate != None:
            self.baudrate = baudrate
        try:
            self.serial_device = serial.Serial(self.device_port, baudrate=self.baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_TWO, timeout=None)
            return self.serial_device
        except serial.SerialException as e:
            print("Error opening serial port {}: {}".format(device_port, e))
            return None

    def adapter_close(self) -> None:
        """Closes the serial port connection."""
        try:
            if self.serial_device is not None and hasattr(self.serial_device, 'close'):
                self.serial_device.close()
        except serial.SerialException as e:
            print("Error closing serial port".format(e))
            return None

    @staticmethod
    def hex_value(c: str) -> int:
        """Converts a hexadecimal character to its integer value."""
        if '0' <= c <= '9':
            return ord(c) - ord('0')
        elif 'A' <= c <= 'F':
            return ord(c) - ord('A') + 10
        elif 'a' <= c <= 'f':
            return ord(c) - ord('a') + 10
        else:
            return -1

    def convert_from_hex(self, hex_string: str) -> bytearray:
        """Converts a hexadecimal string to a bytearray."""
        bin_string = bytearray()
        high = None

        for c in hex_string:
            value = self.hex_value(c)
            if value >= 0:
                if high is None:
                    high = value
                else:
                    bin_string.append(high * 16 + value)
                    high = None

        if len(bin_string) > len(hex_string) // 2:
            print(f"hex string truncated to {len(bin_string)} bytes")

        return bin_string

    @staticmethod
    def display_help() -> None:
        """Displays the help message for command-line arguments."""
        print(f"Usage: {sys.argv[0]} <options>")
        print("Options:")
        print("  -h          Display this help and exit.")
        print("  -t          Print TTY/serial traffic debugging info.")
        print(f"  -d DEVICE   Use TTY DEVICE.")
        print("  -s SPEED    Set CAN SPEED in bps.")
        print(f"  -b BAUDRATE Set TTY/serial BAUDRATE (default: {UsbCanAdapter.CANUSB_TTY_BAUD_RATE_DEFAULT}).")
        print("  -i ID       Inject using ID (specified as hex string).")
        print("  -j DATA     CAN DATA to inject (specified as hex string).")
        print(f"  -n COUNT    Terminate after COUNT frames (default: infinite).")
        print(f"  -g MS       Inject sleep gap in MS milliseconds (default: {UsbCanAdapter.CANUSB_INJECT_SLEEP_GAP_DEFAULT} ms).")
        print(f"  -m MODE     Inject payload MODE ({CANUSB_PAYLOAD_MODE.INJECT_PAYLOAD_MODE_RANDOM.value} = random, {CANUSB_PAYLOAD_MODE.INJECT_PAYLOAD_MODE_INCREMENTAL.value} = incremental, {CANUSB_PAYLOAD_MODE.INJECT_PAYLOAD_MODE_FIXED.value} = fixed).")
        print()


    def sigterm(self, signo, frame) -> None:
        """Handles termination signals to gracefully shut down the program."""
        self.program_running = 0

    def set_can_baudrate(self, baudrate: int) -> None:
        """Sets the CAN baud rate."""
        self.speed = self.canusb_int_to_speed(baudrate)

    def set_port(self, port: str) -> None:
        """Sets the device port."""
        self.device_port = port

    def main(self) -> None:

        self.print_traffic = False

        baudrate = UsbCanAdapter.CANUSB_TTY_BAUD_RATE_DEFAULT

        signal.signal(signal.SIGTERM, self.sigterm)
        #signal.signal(signal.SIGHUP, self.sigterm)
        signal.signal(signal.SIGINT, self.sigterm)

        
        tty_device = "/dev/ttyUSB0" # Adjust to match your serial port
        

        # if speed == 0:
        #     print("Please specify a valid speed!")
        #     self.display_help()
        #     sys.exit(1)

        self.adapter_init(tty_device, baudrate)
        if self.serial_device is None:
            sys.exit(1)

        self.command_settings(1000000, CANUSB_MODE.NORMAL, CANUSB_FRAME.STANDARD)

        self.dump_data_frames(print_flag = False)

        sys.exit(0)


if __name__ == "__main__":
    print('Initialising receiver')
    try:
        while True:
                uca = UsbCanAdapter()
                uca.main()
    except KeyboardInterrupt:
        pass 