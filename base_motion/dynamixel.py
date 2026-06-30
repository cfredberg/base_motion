import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from std_msgs.msg import Int8

import serial

import struct

FRONT_LEFT_ADDR = 0
BACK_LEFT_ADDR = 2
FRONT_RIGHT_ADDR = 1
BACK_RIGHT_ADDR = 3

BROUD_PERCENT = .2

TIGHT_PERCENT = .6

class BaseMotion(Node):

    def __init__(self):
        super().__init__('base_motion')

        self.dyna_controller = DynamixelController("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4TFUT7-if00-port0")

        self.front_left = Dynamixel("front_left", FRONT_LEFT_ADDR, self.dyna_controller)
        self.back_left = Dynamixel("back_left", BACK_LEFT_ADDR, self.dyna_controller)
        self.front_right = Dynamixel("front_right", FRONT_RIGHT_ADDR, self.dyna_controller)
        self.back_right = Dynamixel("back_right", BACK_RIGHT_ADDR, self.dyna_controller)

        self.speed_mult = 0.5

        self.motors_arr = [self.front_left, self.back_left, self.front_right, self.back_right]

        self.base_motion_sub = self.create_subscription(
            String,
            '/motor_states/drive',
            self.drive_direction,
            1)
        self.base_motion_sub

        self.speed_sub = self.create_subscription(
            Int8,
            '/speed',
            self.get_speed,
            1)
        self.speed_sub

        self.auto_sub = self.create_subscription(
            Bool,
            '/auto_on',
            self.set_auto,
            1)
        self.auto_sub

        self.direction = "still"

        # timer_period = 1/30
        # self.timer = self.create_timer(timer_period, self.send_motor_cmds)

    def drive_direction(self, direction_msg):
        direction = direction_msg.data

        if self.direction != direction:
            self.direction = direction
        
            if direction == "forward":
                self.front_left.direction_mult = 1.0
                self.back_left.direction_mult = 1.0
                self.front_right.direction_mult = -1.0
                self.back_right.direction_mult = -1.0
            elif direction == "reverse":
                self.front_left.direction_mult = -1.0
                self.back_left.direction_mult = -1.0
                self.front_right.direction_mult = 1.0
                self.back_right.direction_mult = 1.0
            elif direction == "forward_left":
                self.front_left.direction_mult = 1.0*BROUD_PERCENT
                self.back_left.direction_mult = 1.0*BROUD_PERCENT
                # self.front_left.direction_mult = 0
                # self.back_left.direction_mult = 0
                self.front_right.direction_mult = -1.0
                self.back_right.direction_mult = -1.0
            elif direction == "forward_right":
                self.front_left.direction_mult = 1.0
                self.back_left.direction_mult = 1.0
                self.front_right.direction_mult = -1.0*BROUD_PERCENT
                self.back_right.direction_mult = -1.0*BROUD_PERCENT
                # self.front_right.direction_mult = 0
                # self.back_right.direction_mult = 0
            elif direction == "reverse_left":
                self.front_left.direction_mult = -1.0*BROUD_PERCENT
                self.back_left.direction_mult = -1.0*BROUD_PERCENT
                # self.front_left.direction_mult = 0
                # self.back_left.direction_mult = 0
                self.front_right.direction_mult = 1.0
                self.back_right.direction_mult = 1.0
            elif direction == "reverse_right":
                self.front_left.direction_mult = -1.0
                self.back_left.direction_mult = -1.0
                # self.front_right.direction_mult = 0
                # self.back_right.direction_mult = 0
                self.front_right.direction_mult = 1.0*BROUD_PERCENT
                self.back_right.direction_mult = 1.0*BROUD_PERCENT
            elif direction == "left":
                self.front_left.direction_mult = -TIGHT_PERCENT
                self.back_left.direction_mult = -TIGHT_PERCENT
                self.front_right.direction_mult = -TIGHT_PERCENT
                self.back_right.direction_mult = -TIGHT_PERCENT
            elif direction == "right":
                self.front_left.direction_mult = TIGHT_PERCENT
                self.back_left.direction_mult = TIGHT_PERCENT
                self.front_right.direction_mult = TIGHT_PERCENT
                self.back_right.direction_mult = TIGHT_PERCENT
            else:
                # direction == "still"
                self.front_left.direction_mult = 0.0
                self.back_left.direction_mult = 0.0
                self.front_right.direction_mult = 0.0
                self.back_right.direction_mult = 0.0
            
            self.send_motor_cmds()
    
    def send_motor_cmds(self):
        for motor in self.motors_arr:
            # send motor commands
            speed = motor.direction_mult*self.speed_mult
            print(f"Set {motor.name} speed: {speed}")
            motor.set_speed(speed)

    def get_speed(self, speed_msg):
        speed = self.speed_mult*100
        if speed != speed_msg.data:
            self.speed_mult = speed_msg.data/100
            self.send_motor_cmds()

    def set_auto(self, auto_msg):
        auto = auto_msg.data
        if auto:
            BROUD_PERCENT = 0.8
        else:
            BROUD_PERCENT = 0.2


def main(args=None):
    rclpy.init(args=args)

    base_motion = BaseMotion()

    rclpy.spin(base_motion)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    base_motion.destroy_node()
    rclpy.shutdown()

import dynamixel_sdk
import time
class DynamixelController:
    def __init__(self,dev: str):
        self.port_handler = dynamixel_sdk.PortHandler(dev)
        self.packet_handler = dynamixel_sdk.PacketHandler(2.0)
        self.port_handler.openPort()
        self.port_handler.setBaudRate(1000000)
    def toggle_torque(self, addr: int, value: int):
        return self.packet_handler.write1ByteTxRx(self.port_handler, addr, 64, value)[0]
    def set_mode_run(self, addr: int):
        self.packet_handler.write1ByteTxRx(self.port_handler, addr, 11, 1)
    # returns true on error
    def set_speed(self, addr: int, speed: float):
        return self.packet_handler.write4ByteTxRx(self.port_handler, addr, 104, int(speed * 265))[1] != 0
    def reboot(self,addr: int):
        self.packet_handler.reboot(self.port_handler, addr)
        while self.toggle_torque(addr, 1) != 0: time.sleep(0.01)
class Dynamixel:
    def __init__(self, name: str, motor_addr: int, controller: DynamixelController):
        self.controller = controller
        self.name = name
        self.motor_addr = motor_addr
        self.controller.toggle_torque(self.motor_addr, 0)
        self.controller.set_mode_run(self.motor_addr)
        self.controller.toggle_torque(self.motor_addr, 1)
        self.direction_mult = 0.0
    def set_speed(self, speed: float):
        return self.controller.set_speed(self.motor_addr, speed)
    def reboot(self):
        self.controller.reboot(self.motor_addr)



if __name__ == '__main__':
    main()