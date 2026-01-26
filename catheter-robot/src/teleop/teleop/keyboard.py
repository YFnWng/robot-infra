# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import threading
# from pynput import keyboard

from teleop_interface.msg import TeleopIntents
import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy
from rclpy.node import Node

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %.2f\tturn %.2f ' % (speed, turn)

class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__(
            'keyboard',
            automatically_declare_parameters_from_overrides=True
            )

        # --- terminal ---
        self.settings = saveTerminalSettings()

        # --- parameters ---
        # read_only = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
        self.key_bindings = self.get_parameter('key_bindings').value
        self.vel = self.get_parameter('velocities').value

        self.add_on_set_parameters_callback(
            self._on_parameter_update
        )

        # --- message type ---
        self.teleop_msg = TeleopIntents()

        # --- publisher ---
        self.pub = self.create_publisher(TeleopIntents, 'teleop_cmd', 10)

        # --- state ---
        # self.x = self.y = self.z = self.th = 0.0
        self.status = 0

        # Currently pressed keys
        self.pressed_keys = set()
        self.lock = threading.Lock()

        # Start keyboard listener in a thread
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()

        # Spin publishing thread
        self.timer = self.create_timer(0.05, self.publish_teleop)  # 20 Hz

        # --- spin thread ---
        self._spinner = threading.Thread(
            target=rclpy.spin,
            args=(self,),
            daemon=True
        )
        self._spinner.start()

    # Keyboard callbacks
    def on_press(self, key):
        try:
            k = key.char
        except AttributeError:
            # special keys like shift, ctrl, arrows
            k = str(key)
        with self.lock:
            self.pressed_keys.add(k)

    def on_release(self, key):
        try:
            k = key.char
        except AttributeError:
            k = str(key)
        with self.lock:
            if k in self.pressed_keys:
                self.pressed_keys.remove(k)
        # Exit on Ctrl+C
        if key == keyboard.Key.esc:
            rclpy.shutdown()

    # Compute current motion based on pressed keys
    def compute_motion(self):
        x = y = z = th = 0.0
        with self.lock:
            for k in self.pressed_keys:
                if k in self.move_bindings:
                    dx, dy, dz, dth = self.move_bindings[k]
                    x += dx
                    y += dy
                    z += dz
                    th += dth
                elif k in self.speed_bindings:
                    self.speed *= self.speed_bindings[k][0]
                    self.turn *= self.speed_bindings[k][1]
        return x, y, z, th

    # Compute current motion based on pressed keys and publish
    def publish_teleop(self):
        self.teleop_msg.header.stamp = \
            self.get_clock().now().to_msg()

        self.teleop_msg.catheter_lin_vel = self.vel['catheter_lin'] * catheter_lin_scale
        self.teleop_msg.catheter_rot_vel = self.vel['catheter_rot'] * catheter_rot_scale
        self.teleop_msg.catheter_bend_vel = self.vel['catheter_bend'] * catheter_bend_scale
        self.teleop_msg.sheath_lin_vel = self.vel['sheath_lin'] * sheath_lin_scale
        self.teleop_msg.sheath_rot_vel = self.vel['sheath_rot'] * sheath_rot_scale
        self.teleop_msg.sheath_bend_vel = self.vel['sheath_bend'] * sheath_bend_scale

        self.pub.publish(self.teleop_msg)

    def run(self):
        try:
            print(msg)
            print(self.velocities)

            while rclpy.ok():
                key = getKey(self.settings)

                if key in self.key_bindings:
                    catheter_lin_scale = catheter_rot_scale = catheter_bend_scale = \
                    sheath_lin_scale = sheath_rot_scale = sheath_bend_scale = 0.0
                else:
                    catheter_lin_scale = catheter_rot_scale = catheter_bend_scale = \
                    sheath_lin_scale = sheath_rot_scale = sheath_bend_scale = 0.0
                    if key == '\x03':
                        break

                self.teleop_msg.header.stamp = \
                    self.get_clock().now().to_msg()

                self.teleop_msg.catheter_lin_vel = self.vel['catheter_lin'] * catheter_lin_scale
                self.teleop_msg.catheter_rot_vel = self.vel['catheter_rot'] * catheter_rot_scale
                self.teleop_msg.catheter_bend_vel = self.vel['catheter_bend'] * catheter_bend_scale
                self.teleop_msg.sheath_lin_vel = self.vel['sheath_lin'] * sheath_lin_scale
                self.teleop_msg.sheath_rot_vel = self.vel['sheath_rot'] * sheath_rot_scale
                self.teleop_msg.sheath_bend_vel = self.vel['sheath_bend'] * sheath_bend_scale

                self.pub.publish(self.teleop_msg)

        except Exception as e:
            print(e)

        finally:
            self.stop()

    def stop(self):
        if self.stamped:
            self.twist_msg.header.stamp = \
                self.get_clock().now().to_msg()

        self.teleop_msg.catheter_lin_vel = 0.0
        self.teleop_msg.catheter_rot_vel = 0.0
        self.teleop_msg.catheter_bend_vel = 0.0
        self.teleop_msg.sheath_lin_vel = 0.0
        self.teleop_msg.sheath_rot_vel = 0.0
        self.teleop_msg.sheath_bend_vel = 0.0

        self.pub.publish(self.teleop_msg)
        restoreTerminalSettings(self.settings)

def main():
    rclpy.init()
    node = TeleopKeyboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
