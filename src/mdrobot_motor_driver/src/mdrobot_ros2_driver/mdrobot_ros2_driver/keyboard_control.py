import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class KeyboardMotorControl(Node):

    def __init__(self):
        super().__init__('keyboard_motor_control')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/mdrobot_motor_driver/cmd_velocity',
            10
        )

        # Speed settings
        self.start_speed = 10.0
        self.speed_step = 10.0
        self.max_speed = 50.0

        # Turning speed ratio
        # 0.5 means the inside wheel runs at 50% speed
        self.turn_ratio = 0.5

        self.speed = 0.0

        # Remember the previous forward/backward direction
        self.current_direction = None

        # Current steering direction
        self.current_turn = None

        self.settings = termios.tcgetattr(sys.stdin)

        self.timer = self.create_timer(
            0.1,
            self.publish_velocity
        )

        print()
        print('================================')
        print('      MD200T Keyboard Control')
        print('================================')
        print(' W : Forward')
        print(' S : Backward')
        print(' A : Left')
        print(' D : Right')
        print(' X : Stop')
        print(' Q : Quit')
        print('================================')
        print(f' Start speed : {self.start_speed}')
        print(f' Speed step  : {self.speed_step}')
        print(f' Max speed   : {self.max_speed}')
        print(f' Turn ratio  : {self.turn_ratio}')
        print('================================')
        print()

    def publish_velocity(self):

        # --------------------------------
        # Forward
        # --------------------------------
        if self.current_direction == 'w':

            # Forward + Left
            if self.current_turn == 'd':

                left = -self.speed * self.turn_ratio
                right = -self.speed

            # Forward + Right
            elif self.current_turn == 'a':

                left = -self.speed
                right = -self.speed * self.turn_ratio

            # Forward straight
            else:

                left = -self.speed
                right = -self.speed

        # --------------------------------
        # Backward
        # --------------------------------
        elif self.current_direction == 's':

            # Backward + Left
            if self.current_turn == 'a':

                left = self.speed * self.turn_ratio
                right = self.speed

            # Backward + Right
            elif self.current_turn == 'd':

                left = self.speed
                right = self.speed * self.turn_ratio

            # Backward straight
            else:

                left = self.speed
                right = self.speed

        # --------------------------------
        # Stop
        # --------------------------------
        else:

            left = 0.0
            right = 0.0

        msg = Float64MultiArray()
        msg.data = [left, right]

        self.publisher.publish(msg)

    def get_key(self):

        tty.setraw(sys.stdin.fileno())

        rlist, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0.1
        )

        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.settings
        )

        return key

    def change_direction(self, direction):

        # Same direction → increase speed
        if self.current_direction == direction:

            self.speed += self.speed_step

            if self.speed > self.max_speed:
                self.speed = self.max_speed

        # New direction → start from start_speed
        else:

            self.current_direction = direction
            self.speed = self.start_speed

    def change_turn(self, turn):

        # Same turn → increase speed
        if self.current_turn == turn:

            self.speed += self.speed_step

            if self.speed > self.max_speed:
                self.speed = self.max_speed

        # New turn → keep current speed
        else:

            self.current_turn = turn

    def stop(self):

        self.current_direction = None
        self.current_turn = None
        self.speed = 0.0

        self.publish_velocity()

    def run(self):

        try:

            while rclpy.ok():

                key = self.get_key().lower()

                # --------------------------------
                # Forward
                # --------------------------------
                if key == 'w':

                    self.change_direction('w')

                    # W cancels steering
                    self.current_turn = None

                    print(
                        f'\rFORWARD  speed={self.speed:.0f}    ',
                        end='',
                        flush=True
                    )

                # --------------------------------
                # Backward
                # --------------------------------
                elif key == 's':

                    self.change_direction('s')

                    # S cancels steering
                    self.current_turn = None

                    print(
                        f'\rBACKWARD speed={self.speed:.0f}    ',
                        end='',
                        flush=True
                    )

                # --------------------------------
                # Left
                # --------------------------------
                elif key == 'a':

                    if self.current_direction in ['w', 's']:

                        self.change_turn('a')

                        print(
                            f'\rLEFT     speed={self.speed:.0f}    ',
                            end='',
                            flush=True
                        )

                    else:

                        self.change_direction('a')

                        print(
                            f'\rLEFT     speed={self.speed:.0f}    ',
                            end='',
                            flush=True
                        )

                # --------------------------------
                # Right
                # --------------------------------
                elif key == 'd':

                    if self.current_direction in ['w', 's']:

                        self.change_turn('d')

                        print(
                            f'\rRIGHT    speed={self.speed:.0f}    ',
                            end='',
                            flush=True
                        )

                    else:

                        self.change_direction('d')

                        print(
                            f'\rRIGHT    speed={self.speed:.0f}    ',
                            end='',
                            flush=True
                        )

                # --------------------------------
                # Stop
                # --------------------------------
                elif key == 'x':

                    self.stop()

                    print(
                        '\rSTOP                  ',
                        end='',
                        flush=True
                    )

                # --------------------------------
                # Quit
                # --------------------------------
                elif key == 'q':

                    self.stop()

                    print('\rQUIT')

                    break

                rclpy.spin_once(
                    self,
                    timeout_sec=0.01
                )

        finally:

            self.stop()

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.settings
            )


def main(args=None):

    rclpy.init(args=args)

    node = KeyboardMotorControl()

    try:

        node.run()

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()