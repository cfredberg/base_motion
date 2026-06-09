import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int8

import serial

FRONT_LEFT_ADDR = 0
BACK_LEFT_ADDR = 1
FRONT_RIGHT_ADDR = 2
BACK_RIGHT_ADDR = 3

BROUD_PERCENT = .5

TIGHT_PERCENT = .6

class BaseMotion(Node):

    def __init__(self):
        super().__init__('base_motion')

        self.declare_parameter("nano_addr", "/dev/ttyACM0")

        self.get_parameter("nano_addr").get_parameter_value().string_value

        self.front_left = CheetahMotor("front_left", FRONT_LEFT_ADDR)
        self.back_left = CheetahMotor("back_left", BACK_LEFT_ADDR)
        self.front_right = CheetahMotor("front_right", FRONT_RIGHT_ADDR)
        self.back_right = CheetahMotor("back_right", BACK_RIGHT_ADDR)

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

        timer_period = 1/60
        self.timer = self.create_timer(timer_period, self.send_motor_cmds)

    def drive_direction(self, direction_msg):
        direction = direction_msg.data
        
        if direction == "forward":
            self.front_left.direction_mult = -1.0
            self.back_left.direction_mult = -1.0
            self.front_right.direction_mult = 1.0
            self.back_right.direction_mult = 1.0
        elif direction == "reverse":
            self.front_left.direction_mult = 1.0
            self.back_left.direction_mult = 1.0
            self.front_right.direction_mult = -1.0
            self.back_right.direction_mult = -1.0
        elif direction == "forward_left":
            self.front_left.direction_mult = -1.0*BROUD_PERCENT
            self.back_left.direction_mult = -1.0*BROUD_PERCENT
            self.front_right.direction_mult = 1.0
            self.back_right.direction_mult = 1.0
        elif direction == "forward_right":
            self.front_left.direction_mult = -1.0
            self.back_left.direction_mult = -1.0
            self.front_right.direction_mult = 1.0*BROUD_PERCENT
            self.back_right.direction_mult = 1.0*BROUD_PERCENT
        elif direction == "reverse_left":
            self.front_left.direction_mult = 1.0*BROUD_PERCENT
            self.back_left.direction_mult = 1.0*BROUD_PERCENT
            self.front_right.direction_mult = -1.0
            self.back_right.direction_mult = -1.0
        elif direction == "reverse_right":
            self.front_left.direction_mult = 1.0
            self.back_left.direction_mult = 1.0
            self.front_right.direction_mult = -1.0*BROUD_PERCENT
            self.back_right.direction_mult = -1.0*BROUD_PERCENT
        elif direction == "left":
            self.front_left.direction_mult = TIGHT_PERCENT
            self.back_left.direction_mult = TIGHT_PERCENT
            self.front_right.direction_mult = -TIGHT_PERCENT
            self.back_right.direction_mult = -TIGHT_PERCENT
        elif direction == "right":
            self.front_left.direction_mult = -TIGHT_PERCENT
            self.back_left.direction_mult = -TIGHT_PERCENT
            self.front_right.direction_mult = TIGHT_PERCENT
            self.back_right.direction_mult = TIGHT_PERCENT
        else:
            # direction == "still"
            self.front_left.direction_mult = 0.0
            self.back_left.direction_mult = 0.0
            self.front_right.direction_mult = 0.0
            self.back_right.direction_mult = 0.0
    
    def send_motor_cmds(self):
        for motor in self.motors_arr:
            # send motor commands
            speed = motor.direction_mult*self.speed_mult
            print(f"Set {motor.name} speed: {speed}")
            motor.set_speed(speed)

    def get_speed(self, speed_msg):
        self.speed = speed_msg.data


def main(args=None):
    rclpy.init(args=args)

    base_motion = BaseMotion()

    rclpy.spin(base_motion)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    base_motion.destroy_node()
    rclpy.shutdown()

class CheetahMotor:
    def __init__(self, name: str, motor_addr: int):
        self.name = name
        self.motor_addr = motor_addr
        self.direction_mult = 0.0

        self.ser = serial.Serial(self.nano_addr, 9600) #for example

    def set_speed(self, speed: int):
        self.ser.write(struct.pack('=bfb', ord('S'), speed, self.motor_addr))



if __name__ == '__main__':
    main()