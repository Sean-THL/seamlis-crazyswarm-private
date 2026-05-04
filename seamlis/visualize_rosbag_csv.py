#!/usr/bin/env python3
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Polygon, Rectangle

from examples.flylab import (
    DASC_X_MAX,
    DASC_X_MIN,
    DASC_Y_MAX,
    DASC_Y_MIN,
    UNKNOWN_OBS_VICON_X,
    UNKNOWN_OBS_VICON_Y,
    get_robot_specs,
)


DEFAULT_BAG_DIR = "/root/crazyswarm/bags/gatekeeper"
DEFAULT_ROBOT_SPEC = get_robot_specs(1, use_astar=True)[0]
DEFAULT_FOV_ANGLE_DEG = float(DEFAULT_ROBOT_SPEC.get("fov_angle", 70.0))
DEFAULT_CAM_RANGE = float(DEFAULT_ROBOT_SPEC.get("cam_range", 0.8))
VISUALIZATION_UNKNOWN_OBS_RADIUS = 0.4


def _load_csv(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None

    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=float)
    if data.size == 0:
        return None
    if data.ndim == 0:
        return None
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 7:
        return None
    return {
        "time": data[:, 0],
        "x": data[:, 4],
        "y": data[:, 5],
        "z": data[:, 6],
        "yaw": data[:, 7] if data.shape[1] > 7 else np.zeros(data.shape[0]),
    }


def _downsample_series(data, stride):
    if data is None:
        return None
    stride = max(int(stride), 1)
    if stride == 1:
        return data
    indices = np.arange(0, len(data["x"]), stride, dtype=int)
    if indices.size == 0 or indices[-1] != len(data["x"]) - 1:
        indices = np.append(indices, len(data["x"]) - 1)
    return {key: np.asarray(value)[indices] for key, value in data.items()}


def _field(data, name):
    if data is None:
        return np.empty(0, dtype=float)

    aliases = {
        "%time": "time",
        "time": "time",
        "field.x": "x",
        "fieldx": "x",
        "x": "x",
        "field.y": "y",
        "fieldy": "y",
        "y": "y",
        "field.z": "z",
        "fieldz": "z",
        "z": "z",
        "field.yaw": "yaw",
        "fieldyaw": "yaw",
        "yaw": "yaw",
    }
    key = aliases.get(name, name)
    if key in data:
        return np.asarray(data[key], dtype=float)
    return np.empty(0, dtype=float)


def _time_seconds(data):
    raw_time = _field(data, "time")
    if raw_time.size == 0:
        return raw_time
    return (raw_time - raw_time[0]) * 1e-9


def _load_robot_series(bag_dir, cf_id, state_stride=1, cmd_stride=1):
    state = _load_csv(os.path.join(bag_dir, f"cf{cf_id}_state.csv"))
    cmd = _load_csv(os.path.join(bag_dir, f"cf{cf_id}_cmd_position.csv"))
    state = _downsample_series(state, state_stride)
    cmd = _downsample_series(cmd, cmd_stride)
    return {
        "state": state,
        "cmd": cmd,
        "state_t": _time_seconds(state),
        "cmd_t": _time_seconds(cmd),
    }


def _fov_points(x, y, yaw, fov_angle_deg, cam_range, resolution=24):
    half_angle = np.deg2rad(float(fov_angle_deg)) / 2.0
    angles = np.linspace(float(yaw) - half_angle, float(yaw) + half_angle, int(resolution))
    arc = np.column_stack((
        float(x) + float(cam_range) * np.cos(angles),
        float(y) + float(cam_range) * np.sin(angles),
    ))
    return np.vstack((np.array([[float(x), float(y)]]), arc))


def _heading_series(x, y, yaw, mode):
    if x.size == 0:
        return np.empty(0, dtype=float)
    if mode == "yaw" and yaw.size == x.size:
        return yaw

    heading = np.zeros_like(x, dtype=float)
    if yaw.size == x.size:
        heading[:] = yaw
    for idx in range(1, x.size):
        dx = x[idx] - x[idx - 1]
        dy = y[idx] - y[idx - 1]
        if np.hypot(dx, dy) > 1e-4:
            heading[idx] = np.arctan2(dy, dx)
        else:
            heading[idx] = heading[idx - 1]
    if mode == "velocity" and x.size > 1:
        heading[0] = heading[1]
    return heading


def _add_fov_patch(ax, x, y, yaw, color, fov_angle_deg, cam_range, alpha=0.12, zorder=3):
    patch = Polygon(
        _fov_points(x, y, yaw, fov_angle_deg, cam_range),
        closed=True,
        edgecolor=color,
        facecolor=color,
        alpha=alpha,
        linewidth=1.0,
        zorder=zorder,
    )
    ax.add_patch(patch)
    return patch


def _downsample_indices(size, stride):
    if size <= 0:
        return []
    indices = list(range(0, size, max(int(stride), 1)))
    if indices[-1] != size - 1:
        indices.append(size - 1)
    return indices


def _print_series_summary(cf_id, series):
    for label in ("state", "cmd"):
        data = series[label]
        x = _field(data, "x")
        y = _field(data, "y")
        if x.size == 0 or y.size == 0:
            print(f"cf{cf_id} {label}: no data")
            continue
        print(
            f"cf{cf_id} {label}: {x.size} samples, "
            f"x=[{x.min():.3f}, {x.max():.3f}], "
            f"y=[{y.min():.3f}, {y.max():.3f}]"
        )


def _set_axes(ax, title):
    ax.set_title(title)
    ax.set_xlabel("Vicon x [m]")
    ax.set_ylabel("Vicon y [m]")
    ax.set_aspect("equal", adjustable="box")

    width = DASC_X_MAX - DASC_X_MIN
    height = DASC_Y_MAX - DASC_Y_MIN
    ax.add_patch(
        Rectangle(
            (DASC_X_MIN, DASC_Y_MIN),
            width,
            height,
            edgecolor="black",
            facecolor="none",
            linewidth=2.0,
            label="Flylab boundary",
        )
    )
    ax.add_patch(
        Circle(
            (UNKNOWN_OBS_VICON_X, UNKNOWN_OBS_VICON_Y),
            VISUALIZATION_UNKNOWN_OBS_RADIUS,
            edgecolor="black",
            facecolor="tab:orange",
            alpha=0.35,
            label="Unknown obstacle",
        )
    )

    margin = 0.35
    ax.set_xlim(DASC_X_MIN - margin, DASC_X_MAX + margin)
    ax.set_ylim(DASC_Y_MIN - margin, DASC_Y_MAX + margin)
    ax.grid(True, alpha=0.25)


def plot_static(bag_dir, cf_ids, show_cmd, show_fov, show_sensed_area, fov_angle_deg, cam_range, fov_stride, fov_heading, state_stride, cmd_stride):
    fig, ax = plt.subplots()
    _set_axes(ax, f"SEAMLiS rosbag trajectory: {os.path.basename(os.path.abspath(bag_dir))}")

    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    plotted = False

    for idx, cf_id in enumerate(cf_ids):
        color = colors[idx % len(colors)]
        series = _load_robot_series(bag_dir, cf_id, state_stride=state_stride, cmd_stride=cmd_stride)
        _print_series_summary(cf_id, series)
        state = series["state"]
        cmd = series["cmd"]

        sx = _field(state, "field.x")
        sy = _field(state, "field.y")
        if sx.size > 0 and sy.size > 0:
            ax.plot(sx, sy, color=color, linewidth=3.0, zorder=5, label=f"cf{cf_id} state")
            ax.scatter(sx[0], sy[0], color=color, marker="o", s=60, zorder=6)
            ax.scatter(sx[-1], sy[-1], color=color, marker="x", s=80, zorder=6)
            if show_fov:
                yaw = _field(state, "yaw")
                heading = _heading_series(sx, sy, yaw, fov_heading)
                if heading.size == sx.size:
                    for sample_idx in _downsample_indices(sx.size, fov_stride):
                        alpha = 0.08 if show_sensed_area else 0.12
                        _add_fov_patch(
                            ax,
                            sx[sample_idx],
                            sy[sample_idx],
                            heading[sample_idx],
                            color,
                            fov_angle_deg,
                            cam_range,
                            alpha=alpha,
                            zorder=2,
                        )
                    _add_fov_patch(
                        ax,
                        sx[-1],
                        sy[-1],
                        heading[-1],
                        color,
                        fov_angle_deg,
                        cam_range,
                        alpha=0.22,
                        zorder=6,
                    )
            plotted = True

        if show_cmd:
            cx = _field(cmd, "field.x")
            cy = _field(cmd, "field.y")
            if cx.size > 0 and cy.size > 0:
                ax.plot(cx, cy, color=color, linestyle="--", linewidth=2.0, alpha=0.75, zorder=4, label=f"cf{cf_id} cmd")
                plotted = True

    if not plotted:
        print(f"No trajectory CSV data found in: {bag_dir}")

    ax.legend(loc="best")
    plt.show()


def _save_animation(animation, output_path, fps):
    output_path = os.path.abspath(os.path.expanduser(output_path))
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    ext = os.path.splitext(output_path)[1].lower()
    writer = "pillow" if ext == ".gif" else "ffmpeg"
    print(f"Saving animation to {output_path} using {writer} writer...")
    animation.save(output_path, writer=writer, fps=float(fps))
    print(f"Saved animation: {output_path}")


def plot_animation(
    bag_dir,
    cf_ids,
    interval_ms,
    show_cmd,
    show_fov,
    show_sensed_area,
    fov_angle_deg,
    cam_range,
    fov_heading,
    sensed_stride,
    playback_stride,
    cmd_stride,
    output,
    fps,
    no_show,
):
    fig, ax = plt.subplots()
    _set_axes(ax, f"SEAMLiS rosbag playback: {os.path.basename(os.path.abspath(bag_dir))}")

    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    series_by_cf = {
        cf_id: _load_robot_series(bag_dir, cf_id, state_stride=playback_stride, cmd_stride=cmd_stride)
        for cf_id in cf_ids
    }
    headings_by_cf = {}
    sensed_indices_by_cf = {}
    sensed_next_by_cf = {}
    sensed_patches_by_cf = {}
    for cf_id, series in series_by_cf.items():
        state = series["state"]
        sx = _field(state, "field.x")
        sy = _field(state, "field.y")
        yaw = _field(state, "yaw")
        headings_by_cf[cf_id] = _heading_series(sx, sy, yaw, fov_heading)
        sensed_indices_by_cf[cf_id] = _downsample_indices(sx.size, sensed_stride)
        sensed_next_by_cf[cf_id] = 0
        sensed_patches_by_cf[cf_id] = []
    max_len = max(
        [
            _field(series["state"], "field.x").size
            for series in series_by_cf.values()
        ]
        + [1]
    )

    artists = {}
    for idx, cf_id in enumerate(cf_ids):
        color = colors[idx % len(colors)]
        state_line, = ax.plot([], [], color=color, linewidth=2.0, label=f"cf{cf_id} state")
        cmd_line, = ax.plot([], [], color=color, linestyle="--", linewidth=1.2, alpha=0.65, label=f"cf{cf_id} cmd")
        cmd_line.set_visible(show_cmd)
        point, = ax.plot([], [], color=color, marker="o", markersize=6)
        fov_patch = None
        if show_fov:
            fov_patch = _add_fov_patch(ax, 0.0, 0.0, 0.0, color, fov_angle_deg, cam_range, alpha=0.18)
            fov_patch.set_visible(False)
        artists[cf_id] = (state_line, cmd_line, point, fov_patch)

    time_text = ax.text(0.02, 0.96, "", transform=ax.transAxes, va="top")
    ax.legend(loc="best")

    def update(frame):
        max_t = 0.0
        updated = [time_text]
        for cf_id, series in series_by_cf.items():
            state = series["state"]
            cmd = series["cmd"]
            state_line, cmd_line, point, fov_patch = artists[cf_id]

            sx = _field(state, "field.x")
            sy = _field(state, "field.y")
            st = series["state_t"]
            heading = headings_by_cf.get(cf_id, np.empty(0, dtype=float))
            n_state = min(frame + 1, sx.size)
            if n_state > 0:
                state_line.set_data(sx[:n_state], sy[:n_state])
                point.set_data([sx[n_state - 1]], [sy[n_state - 1]])
                if fov_patch is not None and heading.size >= n_state:
                    fov_patch.set_xy(
                        _fov_points(sx[n_state - 1], sy[n_state - 1], heading[n_state - 1], fov_angle_deg, cam_range)
                    )
                    fov_patch.set_visible(True)
                if show_sensed_area and heading.size >= n_state:
                    indices = sensed_indices_by_cf[cf_id]
                    while sensed_next_by_cf[cf_id] < len(indices) and indices[sensed_next_by_cf[cf_id]] < n_state:
                        sample_idx = indices[sensed_next_by_cf[cf_id]]
                        sensed_patch = _add_fov_patch(
                            ax,
                            sx[sample_idx],
                            sy[sample_idx],
                            heading[sample_idx],
                            state_line.get_color(),
                            fov_angle_deg,
                            cam_range,
                            alpha=0.055,
                            zorder=1,
                        )
                        sensed_patches_by_cf[cf_id].append(sensed_patch)
                        updated.append(sensed_patch)
                        sensed_next_by_cf[cf_id] += 1
                max_t = max(max_t, float(st[n_state - 1]) if st.size else 0.0)

            if show_cmd:
                cx = _field(cmd, "field.x")
                cy = _field(cmd, "field.y")
                n_cmd = min(frame + 1, cx.size)
                if n_cmd > 0:
                    cmd_line.set_data(cx[:n_cmd], cy[:n_cmd])

            updated.extend([state_line, cmd_line, point])
            if fov_patch is not None:
                updated.append(fov_patch)

        time_text.set_text(f"t = {max_t:.2f} s")
        return updated

    animation = FuncAnimation(fig, update, frames=max_len, interval=interval_ms, blit=False, repeat=False)
    fig._seamlis_animation = animation
    if output:
        _save_animation(animation, output, fps)
    if not no_show:
        plt.show()
    else:
        plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description="Visualize SEAMLiS trajectories extracted from rosbag CSV files.")
    parser.add_argument("--bag_dir", default=DEFAULT_BAG_DIR, help="Directory containing cf<ID>_state.csv and cf<ID>_cmd_position.csv.")
    parser.add_argument("--cf_ids", default="6,7,12", help="Comma-separated Crazyflie IDs to plot.")
    parser.add_argument("--animate", action="store_true", help="Animate the trajectory instead of showing a static plot.")
    parser.add_argument("--interval_ms", type=int, default=60, help="Animation frame interval in milliseconds.")
    parser.add_argument("--output", default=None, help="Save animated playback to this .mp4 or .gif file. Requires --animate.")
    parser.add_argument("--fps", type=float, default=20.0, help="Frames per second when saving animated playback.")
    parser.add_argument("--no_show", action="store_true", help="Do not open a GUI window after saving/showing.")
    parser.add_argument("--show_cmd", action="store_true", help="Show commanded /cmd_position trajectory.")
    parser.add_argument("--show_fov", action="store_true", help="Overlay camera/FOV range.")
    parser.add_argument("--show_sensed_area", action="store_true", help="Accumulate past FOV wedges as sensed area.")
    parser.add_argument("--fov_angle", type=float, default=DEFAULT_FOV_ANGLE_DEG, help="FOV angle in degrees.")
    parser.add_argument("--cam_range", type=float, default=DEFAULT_CAM_RANGE, help="Camera/FOV range in meters.")
    parser.add_argument("--fov_stride", type=int, default=500, help="Static plot sample stride for drawing FOV wedges.")
    parser.add_argument("--sensed_stride", type=int, default=150, help="Animation sample stride for accumulating sensed-area wedges.")
    parser.add_argument("--state_stride", type=int, default=5, help="Static plot state downsample stride.")
    parser.add_argument("--cmd_stride", type=int, default=1, help="Command trajectory downsample stride.")
    parser.add_argument("--playback_stride", type=int, default=20, help="Animation state downsample stride.")
    parser.add_argument(
        "--fov_heading",
        choices=["yaw", "velocity"],
        default="velocity",
        help="Use recorded yaw or trajectory velocity direction for FOV orientation.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    cf_ids = [int(item.strip()) for item in args.cf_ids.split(",") if item.strip()]
    if args.animate:
        plot_animation(
            args.bag_dir,
            cf_ids,
            args.interval_ms,
            args.show_cmd,
            args.show_fov,
            args.show_sensed_area,
            args.fov_angle,
            args.cam_range,
            args.fov_heading,
            args.sensed_stride,
            args.playback_stride,
            args.cmd_stride,
            args.output,
            args.fps,
            args.no_show,
        )
    else:
        if args.output:
            raise SystemExit("--output is only supported with --animate.")
        plot_static(
            args.bag_dir,
            cf_ids,
            args.show_cmd,
            args.show_fov,
            args.show_sensed_area,
            args.fov_angle,
            args.cam_range,
            args.fov_stride,
            args.fov_heading,
            args.state_stride,
            args.cmd_stride,
        )


if __name__ == "__main__":
    main()
