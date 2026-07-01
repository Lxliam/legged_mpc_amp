#!/usr/bin/env python3

import re
import select
import sys
import termios
import time
import tty

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mode_schedule
from std_msgs.msg import Bool


MODE_NAME_TO_NUMBER = {
    "FLY": 0,
    "RH": 1,
    "LH": 2,
    "LH_RH": 3,
    "RF": 4,
    "RF_RH": 5,
    "RF_LH": 6,
    "RF_LH_RH": 7,
    "LF": 8,
    "LF_RH": 9,
    "LF_LH": 10,
    "LF_LH_RH": 11,
    "LF_RF": 12,
    "LF_RF_RH": 13,
    "LF_RF_LH": 14,
    "STANCE": 15,
}


HELP_TEXT = """
Keyboard control
----------------
i       initialize: start controller, gait=stance, zero velocity
k       stop legged controller
space   zero velocity
w/s     increase/decrease forward velocity
a/d     increase/decrease yaw velocity
q/e     increase/decrease lateral velocity
z/x     decrease/increase velocity step
r       reset step size
l       start/stop AMP recording
1-9     publish gait by list index
0       publish stance
?       show this help
Ctrl-C  quit
"""


def extract_block(text, name):
    match = re.search(r"(^|\n)\s*" + re.escape(name) + r"\s*\{", text)
    if not match:
        return None

    start = text.find("{", match.start())
    depth = 0
    for index in range(start, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start + 1:index]
    return None


def indexed_values(block):
    values = []
    for line in block.splitlines():
        line = line.split(";", 1)[0].strip()
        match = re.search(r"\[\s*\d+\s*\]\s+([A-Za-z0-9_.+-]+)", line)
        if match:
            values.append(match.group(1))
    return values


def load_gaits(gait_file):
    with open(gait_file, "r", encoding="utf-8") as handle:
        text = handle.read()

    list_block = extract_block(text, "list")
    if list_block is None:
        raise RuntimeError("Could not find gait list in {}".format(gait_file))

    gait_names = indexed_values(list_block)
    gaits = {}
    for gait_name in gait_names:
        gait_block = extract_block(text, gait_name)
        if gait_block is None:
            rospy.logwarn("Gait '%s' is listed but has no block.", gait_name)
            continue

        mode_block = extract_block(gait_block, "modeSequence")
        time_block = extract_block(gait_block, "switchingTimes")
        if mode_block is None or time_block is None:
            rospy.logwarn("Gait '%s' is missing modeSequence or switchingTimes.", gait_name)
            continue

        modes = []
        for mode_name in indexed_values(mode_block):
            try:
                modes.append(MODE_NAME_TO_NUMBER[mode_name])
            except KeyError as exc:
                raise RuntimeError("Unknown mode '{}' in gait '{}'".format(mode_name, gait_name)) from exc

        event_times = [float(value) for value in indexed_values(time_block)]
        gaits[gait_name] = (event_times, modes)

    return gait_names, gaits


class KeyboardController:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "legged_robot")
        self.rate_hz = rospy.get_param("~rate", 10.0)
        self.linear_step_default = rospy.get_param("~linear_step", 0.10)
        self.yaw_step_default = rospy.get_param("~yaw_step", 0.10)
        self.max_linear_x = rospy.get_param("~max_linear_x", 1.00)
        self.max_linear_y = rospy.get_param("~max_linear_y", 0.50)
        self.max_yaw = rospy.get_param("~max_yaw", 1.00)
        self.linear_ramp_rate = rospy.get_param("~linear_ramp_rate", 0.50)
        self.yaw_ramp_rate = rospy.get_param("~yaw_ramp_rate", 0.80)
        self.gait_settle_time = rospy.get_param("~gait_settle_time", 2.0)
        self.gait_publish_duration = rospy.get_param("~gait_publish_duration", 1.0)
        self.gait_publish_period = rospy.get_param("~gait_publish_period", 0.1)
        self.amp_log_dir = rospy.get_param("/amp_log_dir", "amp_data")
        self.amp_log_prefix = rospy.get_param("/amp_log_prefix", "motion")

        gait_file = rospy.get_param("/gaitCommandFile")
        self.gait_names, self.gaits = load_gaits(gait_file)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.gait_pub = rospy.Publisher(self.robot_name + "_mpc_mode_schedule", mode_schedule, queue_size=1, latch=True)
        self.amp_pub = rospy.Publisher("/amp/enable_logging", Bool, queue_size=1, latch=True)

        self.switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

        self.linear_step = self.linear_step_default
        self.yaw_step = self.yaw_step_default
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_yaw = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.yaw = 0.0
        self.amp_logging = rospy.get_param("~amp_recording_initial", False)
        self.gait_hold_until = 0.0
        self.pending_gait_msg = None
        self.pending_gait_until = 0.0
        self.last_gait_publish = 0.0
        self.amp_pub.publish(Bool(self.amp_logging))

    def clamp(self):
        self.target_linear_x = max(-self.max_linear_x, min(self.max_linear_x, self.target_linear_x))
        self.target_linear_y = max(-self.max_linear_y, min(self.max_linear_y, self.target_linear_y))
        self.target_yaw = max(-self.max_yaw, min(self.max_yaw, self.target_yaw))

    @staticmethod
    def approach(current, target, delta):
        if current < target:
            return min(current + delta, target)
        if current > target:
            return max(current - delta, target)
        return current

    def update_command_ramp(self):
        self.clamp()
        target_x = self.target_linear_x
        target_y = self.target_linear_y
        target_yaw = self.target_yaw

        if time.monotonic() < self.gait_hold_until:
            target_x = 0.0
            target_y = 0.0
            target_yaw = 0.0

        linear_delta = self.linear_ramp_rate / self.rate_hz
        yaw_delta = self.yaw_ramp_rate / self.rate_hz
        self.linear_x = self.approach(self.linear_x, target_x, linear_delta)
        self.linear_y = self.approach(self.linear_y, target_y, linear_delta)
        self.yaw = self.approach(self.yaw, target_yaw, yaw_delta)

    def twist(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = self.linear_y
        msg.angular.z = self.yaw
        return msg

    def publish_cmd(self):
        self.update_command_ramp()
        self.cmd_pub.publish(self.twist())

    def republish_pending_gait(self):
        if self.pending_gait_msg is None:
            return

        now = time.monotonic()
        if now > self.pending_gait_until:
            self.pending_gait_msg = None
            return

        if now - self.last_gait_publish >= self.gait_publish_period:
            self.gait_pub.publish(self.pending_gait_msg)
            self.last_gait_publish = now

    def zero_velocity(self):
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_yaw = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.yaw = 0.0
        self.publish_cmd()
        self.print_status("zero velocity")

    def publish_gait(self, gait_name):
        if gait_name not in self.gaits:
            self.print_status("unknown gait: {}".format(gait_name))
            return

        event_times, modes = self.gaits[gait_name]
        msg = mode_schedule()
        msg.eventTimes = event_times
        msg.modeSequence = modes
        self.zero_velocity()
        self.gait_pub.publish(msg)
        self.pending_gait_msg = msg
        self.pending_gait_until = time.monotonic() + self.gait_publish_duration
        self.last_gait_publish = time.monotonic()
        subscribers = self.gait_pub.get_num_connections()
        if gait_name != "stance":
            self.gait_hold_until = time.monotonic() + self.gait_settle_time
            self.print_status(
                "gait={}, subscribers={}, hold zero velocity for {:.1f}s".format(gait_name, subscribers, self.gait_settle_time)
            )
        else:
            self.gait_hold_until = 0.0
            self.print_status("gait={}, subscribers={}".format(gait_name, subscribers))

    def start_controller(self):
        try:
            rospy.wait_for_service("/controller_manager/switch_controller", timeout=5.0)
            request = SwitchControllerRequest()
            request.start_controllers = ["controllers/joint_state_controller", "controllers/legged_controller"]
            request.stop_controllers = []
            request.strictness = SwitchControllerRequest.BEST_EFFORT
            request.start_asap = False
            request.timeout = 0.0
            response = self.switch_controller(request)
            self.print_status("controller started" if response.ok else "controller start returned false")
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self.print_status("controller start failed: {}".format(exc))

    def stop_controller(self):
        try:
            rospy.wait_for_service("/controller_manager/switch_controller", timeout=5.0)
            request = SwitchControllerRequest()
            request.start_controllers = []
            request.stop_controllers = ["controllers/legged_controller"]
            request.strictness = SwitchControllerRequest.BEST_EFFORT
            request.start_asap = False
            request.timeout = 0.0
            response = self.switch_controller(request)
            self.zero_velocity()
            self.print_status("controller stopped" if response.ok else "controller stop returned false")
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self.print_status("controller stop failed: {}".format(exc))

    def initialize(self):
        self.start_controller()
        self.publish_gait("stance")
        self.zero_velocity()

    def toggle_amp_logging(self):
        self.amp_logging = not self.amp_logging
        self.amp_pub.publish(Bool(self.amp_logging))
        subscribers = self.amp_pub.get_num_connections()
        if self.amp_logging:
            self.print_status(
                "AMP recording start requested, subscribers={}, dir={}, prefix={}".format(
                    subscribers, self.amp_log_dir, self.amp_log_prefix
                )
            )
        else:
            self.print_status("AMP recording stop requested, subscribers={}; see controller terminal for saved file summary".format(subscribers))

    def print_status(self, prefix=""):
        status = "cmd vx={:+.2f} vy={:+.2f} wz={:+.2f} target vx={:+.2f} vy={:+.2f} wz={:+.2f} step={:.2f}/{:.2f}".format(
            self.linear_x,
            self.linear_y,
            self.yaw,
            self.target_linear_x,
            self.target_linear_y,
            self.target_yaw,
            self.linear_step,
            self.yaw_step,
        )
        if prefix:
            print("[{}] {}".format(prefix, status))
        else:
            print(status)

    def print_help(self):
        print(HELP_TEXT)
        print("Available gaits:")
        if "stance" in self.gait_names:
            print("  0: stance")
        for index, gait_name in enumerate([name for name in self.gait_names if name != "stance"][:9], start=1):
            print("  {}: {}".format(index, gait_name))
        self.print_status()

    def handle_key(self, key):
        if time.monotonic() < self.gait_hold_until and key in "wsa dqe".replace(" ", ""):
            remaining = self.gait_hold_until - time.monotonic()
            self.print_status("wait {:.1f}s before speed command".format(max(0.0, remaining)))
            return

        if key == "w":
            self.target_linear_x += self.linear_step
        elif key == "s":
            self.target_linear_x -= self.linear_step
        elif key == "q":
            self.target_linear_y += self.linear_step
        elif key == "e":
            self.target_linear_y -= self.linear_step
        elif key == "a":
            self.target_yaw += self.yaw_step
        elif key == "d":
            self.target_yaw -= self.yaw_step
        elif key == " ":
            self.zero_velocity()
            return
        elif key == "i":
            self.initialize()
            return
        elif key == "k":
            self.stop_controller()
            return
        elif key == "l":
            self.toggle_amp_logging()
            return
        elif key == "z":
            self.linear_step = max(0.05, self.linear_step - 0.05)
            self.yaw_step = max(0.05, self.yaw_step - 0.05)
        elif key == "x":
            self.linear_step = min(0.3, self.linear_step + 0.05)
            self.yaw_step = min(0.3, self.yaw_step + 0.05)
        elif key == "r":
            self.linear_step = self.linear_step_default
            self.yaw_step = self.yaw_step_default
        elif key == "?":
            self.print_help()
            return
        elif key == "0":
            self.publish_gait("stance")
            return
        elif key.isdigit():
            gait_options = [name for name in self.gait_names if name != "stance"]
            gait_index = int(key) - 1
            if 0 <= gait_index < len(gait_options):
                self.publish_gait(gait_options[gait_index])
            return
        else:
            return

        self.publish_cmd()
        self.print_status()

    def run(self):
        self.print_help()
        rate = rospy.Rate(self.rate_hz)
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not rospy.is_shutdown():
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if ready:
                    key = sys.stdin.read(1)
                    if key == "\x03":
                        break
                    self.handle_key(key)
                self.republish_pending_gait()
                self.publish_cmd()
                rate.sleep()
        finally:
            self.zero_velocity()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    rospy.init_node("legged_keyboard_control")
    KeyboardController().run()


if __name__ == "__main__":
    main()
