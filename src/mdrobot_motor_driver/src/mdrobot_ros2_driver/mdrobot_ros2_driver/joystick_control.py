import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class XboxMotorControl(Node):

    def __init__(self):
        super().__init__('xbox_motor_control')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/mdrobot_motor_driver/cmd_velocity',
            10
        )

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.steering_axis = 0
        self.throttle_axis = 1

        self.button_a = 0
        self.button_b = 1
        self.button_x = 2
        self.button_lb = 4
        self.button_rb = 5

        self.speed_levels = [10.0, 20.0, 30.0, 40.0, 50.0]
        self.speed_level = 2
        self.current_max_speed = self.speed_levels[self.speed_level]

        self.deadzone = 0.10
        self.steering_gain = 1.0

        self.acceleration = 2.0
        self.deceleration = 4.0

        self.target_left = 0.0
        self.target_right = 0.0

        self.current_left = 0.0
        self.current_right = 0.0

        self.enabled = False

        self.joy_timeout = 0.5
        self.last_joy_time = self.get_clock().now()

        self.previous_buttons = []

        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

        print()
        print('================================')
        print('       Xbox Motor Control')
        print('================================')
        print('A  : Enable')
        print('B  : Stop')
        print('X  : Stop')
        print('LB : Speed -')
        print('RB : Speed +')
        print('================================')
        print(f'Speed: {self.current_max_speed}')
        print('================================')

    def apply_deadzone(self, value):

        if abs(value) < self.deadzone:
            return 0.0

        sign = 1.0 if value >= 0 else -1.0

        value = (
            abs(value) - self.deadzone
        ) / (1.0 - self.deadzone)

        return sign * value

    def button_pressed(self, buttons, index):

        if index >= len(buttons):
            return False

        if index >= len(self.previous_buttons):
            return buttons[index] == 1

        return (
            buttons[index] == 1
            and self.previous_buttons[index] == 0
        )

    def joy_callback(self, msg):

        self.last_joy_time = self.get_clock().now()

        if self.button_pressed(msg.buttons, self.button_a):
            self.enabled = True
            print('\n[ENABLE]')

        if self.button_pressed(msg.buttons, self.button_b):
            self.enabled = False
            self.target_left = 0.0
            self.target_right = 0.0
            self.current_left = 0.0
            self.current_right = 0.0
            print('\n[STOP]')

        if self.button_pressed(msg.buttons, self.button_x):
            self.target_left = 0.0
            self.target_right = 0.0

        if self.button_pressed(msg.buttons, self.button_lb):
            if self.speed_level > 0:
                self.speed_level -= 1
                self.current_max_speed = (
                    self.speed_levels[self.speed_level]
                )
                print(
                    f'\n[SPEED] {self.current_max_speed}'
                )

        if self.button_pressed(msg.buttons, self.button_rb):
            if self.speed_level < len(self.speed_levels) - 1:
                self.speed_level += 1
                self.current_max_speed = (
                    self.speed_levels[self.speed_level]
                )
                print(
                    f'\n[SPEED] {self.current_max_speed}'
                )

        self.previous_buttons = list(msg.buttons)

        if not self.enabled:
            self.target_left = 0.0
            self.target_right = 0.0
            return

        if len(msg.axes) <= max(
            self.steering_axis,
            self.throttle_axis
        ):
            return

        steering = msg.axes[self.steering_axis]
        throttle = msg.axes[self.throttle_axis]

        throttle = self.apply_deadzone(throttle)
        steering = self.apply_deadzone(steering)

        steering *= self.steering_gain

        left = throttle + steering
        right = throttle - steering

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        self.target_left = -left * self.current_max_speed
        self.target_right = -right * self.current_max_speed

    def ramp_speed(self, current, target):

        difference = target - current

        step = (
            self.acceleration
            if abs(target) > abs(current)
            else self.deceleration
        )

        if abs(difference) <= step:
            return target

        return (
            current + step
            if difference > 0
            else current - step
        )

    def control_loop(self):

        elapsed = (
            self.get_clock().now()
            - self.last_joy_time
        ).nanoseconds / 1e9

        if elapsed > self.joy_timeout:
            self.target_left = 0.0
            self.target_right = 0.0
            self.enabled = False

        self.current_left = self.ramp_speed(
            self.current_left,
            self.target_left
        )

        self.current_right = self.ramp_speed(
            self.current_right,
            self.target_right
        )

        msg = Float64MultiArray()

        msg.data = [
            float(self.current_left),
            float(self.current_right)
        ]

        self.publisher.publish(msg)

        print(
            f'\r'
            f'[{ "ON" if self.enabled else "OFF" }] '
            f'SPEED={self.current_max_speed:.0f} '
            f'L={self.current_left:6.1f} '
            f'R={self.current_right:6.1f}',
            end='',
            flush=True
        )

    def stop(self):

        self.enabled = False
        self.target_left = 0.0
        self.target_right = 0.0
        self.current_left = 0.0
        self.current_right = 0.0

        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]

        for _ in range(5):
            self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = XboxMotorControl()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()