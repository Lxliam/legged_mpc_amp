#!/usr/bin/env python3

import argparse
import math
import threading
from collections import deque
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from visualization_msgs.msg import Marker, MarkerArray


FOOT_NAMES = ("LF", "RF", "LH", "RH")
FOOT_COLORS = ("tab:blue", "tab:orange", "gold", "tab:purple")
AXIS_LABELS = {
    "x": "x [m]",
    "y": "y [m]",
    "z": "z [m]",
}


class FootTrajectoryPlotter:
    def __init__(
        self,
        topic: str,
        view: str,
        update_ms: int,
        history: int,
        trail_seconds: float,
        window_size: Tuple[float, float],
        axis_padding: float,
        min_x_span: float,
        combined: bool,
    ):
        self._topic = topic
        self._view = view
        self._combined = combined or view == "3d"
        self._history = max(1, history)
        self._trail_seconds = max(0.0, trail_seconds)
        self._axis_padding = max(0.0, axis_padding)
        self._min_x_span = max(0.0, min_x_span)
        self._lock = threading.Lock()
        self._latest_stamp = None
        self._samples = {name: deque() for name in FOOT_NAMES}
        self._trajectories: Dict[str, List[Tuple[float, float, float]]] = {name: [] for name in FOOT_NAMES}

        self._fig = plt.figure("Optimized foot trajectories", figsize=window_size)
        self._axes = self._make_axes(view)
        self._lines = self._make_lines(view)
        self._format_axes()

        self._subscriber = rospy.Subscriber(topic, MarkerArray, self._callback, queue_size=1)
        self._animation = FuncAnimation(self._fig, self._update_plot, interval=update_ms, cache_frame_data=False)

    def _make_axes(self, view: str):
        if self._combined:
            projection = "3d" if view == "3d" else None
            return {"all": self._fig.add_subplot(111, projection=projection)}

        axes = self._fig.subplots(2, 2, squeeze=False)
        return {name: axes[index // 2][index % 2] for index, name in enumerate(FOOT_NAMES)}

    def _make_lines(self, view: str):
        lines = {}
        for name, color in zip(FOOT_NAMES, FOOT_COLORS):
            axis = self._axes["all"] if self._combined else self._axes[name]
            if view == "3d":
                line, = axis.plot([], [], [], label=name, color=color, linewidth=2.0)
            else:
                line, = axis.plot([], [], label=name, color=color, linewidth=2.0)
            lines[name] = line
        return lines

    def _format_axes(self):
        self._fig.suptitle(self._title())
        for key, axis in self._axes.items():
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper right")
            if not self._combined:
                axis.set_title(key)
            if self._view == "3d":
                axis.set_xlabel(AXIS_LABELS["x"])
                axis.set_ylabel(AXIS_LABELS["y"])
                axis.set_zlabel(AXIS_LABELS["z"])
            elif self._view == "z":
                axis.set_xlabel("trajectory point index")
                axis.set_ylabel(AXIS_LABELS["z"])
            else:
                x_axis, y_axis = self._view
                axis.set_xlabel(AXIS_LABELS[x_axis])
                axis.set_ylabel(AXIS_LABELS[y_axis])
        self._fig.tight_layout()

    def _title(self):
        return f"{self._topic} foot trajectories ({self._view})"

    def _callback(self, msg: MarkerArray):
        ee_markers = [
            marker for marker in msg.markers
            if marker.ns == "EE Trajectories" and marker.type == Marker.LINE_STRIP and marker.points
        ]
        if len(ee_markers) < len(FOOT_NAMES):
            rospy.logwarn_throttle(
                2.0,
                "Waiting for 4 EE Trajectories markers on %s, got %d",
                self._topic,
                len(ee_markers),
            )
            return

        ee_markers = sorted(ee_markers, key=lambda marker: marker.id)[:len(FOOT_NAMES)]
        horizon_trajectories = {}
        current_points = {}
        for name, marker in zip(FOOT_NAMES, ee_markers):
            points = [(point.x, point.y, point.z) for point in marker.points[-self._history:]]
            horizon_trajectories[name] = points
            first_point = marker.points[0]
            current_points[name] = (first_point.x, first_point.y, first_point.z)

        with self._lock:
            self._latest_stamp = msg.markers[0].header.stamp if msg.markers else rospy.Time.now()
            if self._trail_seconds > 0.0:
                now = rospy.get_time()
                for name in FOOT_NAMES:
                    self._samples[name].append((now, current_points[name]))
                    while self._samples[name] and now - self._samples[name][0][0] > self._trail_seconds:
                        self._samples[name].popleft()
                self._trajectories = {
                    name: [point for _stamp, point in self._samples[name]]
                    for name in FOOT_NAMES
                }
            else:
                self._trajectories = horizon_trajectories

    def _update_plot(self, _frame):
        with self._lock:
            trajectories = {name: list(points) for name, points in self._trajectories.items()}
            latest_stamp = self._latest_stamp

        all_x, all_y, all_z = [], [], []
        for name, points in trajectories.items():
            xs = [point[0] for point in points]
            ys = [point[1] for point in points]
            zs = [point[2] for point in points]
            all_x.extend(xs)
            all_y.extend(ys)
            all_z.extend(zs)

            if self._view == "3d":
                self._lines[name].set_data(xs, ys)
                self._lines[name].set_3d_properties(zs)
            elif self._view == "z":
                self._lines[name].set_data(range(len(zs)), zs)
            else:
                axis_values = {"x": xs, "y": ys, "z": zs}
                self._lines[name].set_data(axis_values[self._view[0]], axis_values[self._view[1]])

        self._rescale_axes(trajectories, all_x, all_y, all_z)
        self._fig.suptitle(self._title_with_stamp(latest_stamp))
        return tuple(self._lines.values())

    def _title_with_stamp(self, stamp):
        if stamp is None or stamp == rospy.Time(0):
            return self._title()
        return f"{self._title()}  t={stamp.to_sec():.2f}s"

    def _rescale_axes(self, trajectories, xs, ys, zs):
        if self._view == "3d":
            self._set_limits_3d(trajectories, xs, ys, zs)
        elif self._view == "z":
            max_len = max((len(line.get_ydata()) for line in self._lines.values()), default=1)
            for axis_obj in self._axes.values():
                axis_obj.set_xlim(0, max(1, max_len - 1))
            self._set_limits_2d(zs, axis="y")
        else:
            axis_values = {"x": xs, "y": ys, "z": zs}
            if self._view[0] == "x" and self._min_x_span > 0.0:
                self._set_fixed_x_limits(trajectories)
            else:
                self._set_limits_2d(axis_values[self._view[0]], axis="x")
            self._set_limits_2d(axis_values[self._view[1]], axis="y")

    def _set_limits_2d(self, values, axis: str):
        low, high = padded_limits(values, minimum_span=0.05, padding_ratio=self._axis_padding)
        for axis_obj in self._axes.values():
            if axis == "x":
                axis_obj.set_xlim(low, high)
            else:
                axis_obj.set_ylim(low, high)

    def _set_fixed_x_limits(self, trajectories):
        if self._combined:
            center = mean_or_none(last_finite(point[0] for point in trajectories[name]) for name in FOOT_NAMES)
            low, high = fixed_span_limits(xs_from_trajectories(trajectories), self._min_x_span, center)
            self._axes["all"].set_xlim(low, high)
            return

        for name in FOOT_NAMES:
            center = last_finite(point[0] for point in trajectories[name])
            low, high = fixed_span_limits([point[0] for point in trajectories[name]], self._min_x_span, center)
            self._axes[name].set_xlim(low, high)

    def _set_limits_3d(self, trajectories, xs, ys, zs):
        if self._min_x_span > 0.0:
            center = mean_or_none(last_finite(point[0] for point in trajectories[name]) for name in FOOT_NAMES)
            x_low, x_high = fixed_span_limits(xs, self._min_x_span, center)
        else:
            x_low, x_high = padded_limits(xs, padding_ratio=self._axis_padding)
        y_low, y_high = padded_limits(ys, padding_ratio=self._axis_padding)
        z_low, z_high = padded_limits(zs, padding_ratio=self._axis_padding)
        for axis_obj in self._axes.values():
            axis_obj.set_xlim(x_low, x_high)
            axis_obj.set_ylim(y_low, y_high)
            axis_obj.set_zlim(z_low, z_high)

    def show(self):
        plt.show()


def padded_limits(values, minimum_span=0.05, padding_ratio=0.08):
    clean_values = [value for value in values if math.isfinite(value)]
    if not clean_values:
        return -1.0, 1.0

    low = min(clean_values)
    high = max(clean_values)
    span = max(high - low, minimum_span)
    center = 0.5 * (low + high)
    half_span = 0.5 * span * (1.0 + 2.0 * padding_ratio)
    return center - half_span, center + half_span


def fixed_span_limits(values, span, center=None):
    clean_values = [value for value in values if math.isfinite(value)]
    if center is None or not math.isfinite(center):
        center = 0.5 * (min(clean_values) + max(clean_values)) if clean_values else 0.0
    half_span = 0.5 * max(span, 1e-6)
    return center - half_span, center + half_span


def last_finite(values):
    for value in reversed(list(values)):
        if math.isfinite(value):
            return value
    return None


def mean_or_none(values):
    clean_values = [value for value in values if value is not None and math.isfinite(value)]
    if not clean_values:
        return None
    return sum(clean_values) / len(clean_values)


def xs_from_trajectories(trajectories):
    return [point[0] for name in FOOT_NAMES for point in trajectories[name]]


def parse_args():
    parser = argparse.ArgumentParser(description="Plot optimized quadruped foot trajectories from RViz MarkerArray data.")
    parser.add_argument(
        "--topic",
        default="/legged_robot/optimizedStateTrajectory",
        help="visualization_msgs/MarkerArray topic to subscribe to",
    )
    parser.add_argument(
        "--view",
        choices=("xz", "xy", "yz", "z", "3d"),
        default="xz",
        help="plot view: xz side view, xy top view, yz front view, z height curves, or 3d",
    )
    parser.add_argument("--update-ms", type=int, default=100, help="matplotlib refresh interval in milliseconds")
    parser.add_argument("--history", type=int, default=1000, help="maximum points kept from each incoming marker line")
    parser.add_argument(
        "--trail-seconds",
        type=float,
        default=10.0,
        help="seconds of recent current foot positions to keep as one rolling line; use 0 to show only the newest MPC horizon",
    )
    parser.add_argument(
        "--window-size",
        nargs=2,
        type=float,
        metavar=("WIDTH", "HEIGHT"),
        default=(8.0, 5.0),
        help="matplotlib window size in inches",
    )
    parser.add_argument(
        "--axis-padding",
        type=float,
        default=0.25,
        help="extra axis padding ratio around the visible trajectory data",
    )
    parser.add_argument(
        "--min-x-span",
        type=float,
        default=1.0,
        help="fixed visible x-axis span in meters; use 0 to auto-scale",
    )
    parser.add_argument(
        "--combined",
        action="store_true",
        help="draw all four feet in one axes instead of separate subplots",
    )
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node("foot_trajectory_plotter", anonymous=True)
    plotter = FootTrajectoryPlotter(
        args.topic,
        args.view,
        args.update_ms,
        args.history,
        args.trail_seconds,
        tuple(args.window_size),
        args.axis_padding,
        args.min_x_span,
        args.combined,
    )
    rospy.loginfo("Plotting foot trajectories from %s with view=%s", args.topic, args.view)
    plotter.show()


if __name__ == "__main__":
    main()
