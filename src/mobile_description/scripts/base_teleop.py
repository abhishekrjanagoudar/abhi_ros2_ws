#!/usr/bin/env python3

from __future__ import annotations

import sys
import tty
import termios
import select
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)

from geometry_msgs.msg import Twist


MOVE_BINDINGS = {
    'i': (1, 0),
    ',': (-1, 0),
    'j': (0, 1),
    'l': (0, -1),
    'k': (0, 0),

    'w': (1, 0),
    'x': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    's': (0, 0),

    'I': (1, 0),
    '<': (-1, 0),
    'J': (0, 1),
    'L': (0, -1),

    'W': (1, 0),
    'X': (-1, 0),
    'A': (0, 1),
    'D': (0, -1)
}

SPEED_BINDINGS = {
    'q': (1.1,1.1),
    'z': (0.9,0.9),
    'e': (1.0,1.1),
    'c': (1.0,0.9),

    'Q': (1.1,1.1),
    'Z': (0.9,0.9),
    'E': (1.0,1.1),
    'C': (1.0,0.9)
}


BANNER = """
╔══════════════════════════════════════╗
║ Mobile Robot Teleop                  ║
╠══════════════════════════════════════╣
║ w/i : forward                        ║
║ x/, : backward                       ║
║ a/j : rotate left                    ║
║ d/l : rotate right                   ║
║ s/k : stop                           ║
║ q/z : speed +/-                      ║
║ e/c : angular +/-                    ║
║ Ctrl+C : quit                        ║
╚══════════════════════════════════════╝
"""

SPEED_LINE = (
    "\rLinear: {:.2f} m/s | "
    "Angular: {:.2f} rad/s"
)


def _get_key(timeout=0.05):

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:

        tty.setraw(fd)

        rlist, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            timeout
        )

        return sys.stdin.read(1) if rlist else ''

    finally:

        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old
        )


class BaseTeleop(Node):

    BURST_TIMEOUT_S = 0.15

    def __init__(self):

        super().__init__('base_teleop')

        self.declare_parameter(
            'max_linear_velocity',
            1.0
        )

        self.declare_parameter(
            'max_angular_velocity',
            2.0
        )

        self.max_lin = (
            self.get_parameter(
                'max_linear_velocity'
            ).value
        )

        self.max_ang = (
            self.get_parameter(
                'max_angular_velocity'
            ).value
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/diff_cont/cmd_vel',
            qos
        )

        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self._target_lin = 0.0
        self._target_ang = 0.0

        self._last_key_t = time.monotonic()

        self.running = True

        self.create_timer(
            1.0/20.0,
            self._publish_cb
        )

        self.get_logger().info(
            "Teleop ready"
        )


    def _publish_cb(self):

        if (
            time.monotonic()
            - self._last_key_t
        ) > self.BURST_TIMEOUT_S:

            self._target_lin = 0.0
            self._target_ang = 0.0

        self._publish(
            self._target_lin,
            self._target_ang
        )


    def _publish(self, lin, ang):

        msg = Twist()

        msg.linear.x = float(lin)
        msg.angular.z = float(ang)

        self.cmd_pub.publish(msg)


    def run(self):

        print(BANNER)

        print(
            SPEED_LINE.format(
                self.linear_speed,
                self.angular_speed
            ),
            end='',
            flush=True
        )

        try:

            while rclpy.ok() and self.running:

                key = _get_key()

                if key in MOVE_BINDINGS:

                    lin, ang = MOVE_BINDINGS[key]

                    self._target_lin = (
                        lin * self.linear_speed
                    )

                    self._target_ang = (
                        ang * self.angular_speed
                    )

                    self._last_key_t = (
                        time.monotonic()
                    )


                elif key in SPEED_BINDINGS:

                    lm, am = SPEED_BINDINGS[key]

                    self.linear_speed = round(
                        min(
                            self.max_lin,
                            self.linear_speed * lm
                        ),
                        3
                    )

                    self.angular_speed = round(
                        min(
                            self.max_ang,
                            self.angular_speed * am
                        ),
                        3
                    )

                    print(
                        SPEED_LINE.format(
                            self.linear_speed,
                            self.angular_speed
                        ),
                        end='',
                        flush=True
                    )

                elif key == '\x03':
                    break

        finally:

            for _ in range(5):

                self._publish(
                    0.0,
                    0.0
                )

                time.sleep(0.05)

            self.get_logger().info(
                "Robot stopped"
            )


def main(args=None):

    rclpy.init(args=args)

    node = BaseTeleop()

    thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )

    thread.start()

    try:
        node.run()

    finally:

        node.destroy_node()

        rclpy.shutdown()

        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
