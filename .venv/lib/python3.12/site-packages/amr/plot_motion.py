from __future__ import annotations

import matplotlib.pyplot as plt

from amr.kinematics.params import KinematicsModel
from amr.motion import MotionSession
from amr.simulate import simulate


def motion_bounds(session: MotionSession, *, pad: float = 0.8) -> tuple[float, float, float, float]:
    """Axis limits (xmin, xmax, ymin, ymax) that fit the full trajectory."""
    xs: list[float] = [p.x for p in session.poses]
    ys: list[float] = [p.y for p in session.poses]
    if session.kind == KinematicsModel.MECANUM:
        for tr in (
            session.trace_mecanum_fl,
            session.trace_mecanum_fr,
            session.trace_mecanum_rl,
            session.trace_mecanum_rr,
        ):
            xs += [p[0] for p in tr]
            ys += [p[1] for p in tr]
    elif session.kind in (KinematicsModel.ACKERMANN, KinematicsModel.BICYCLE) and session.trace_front_left:
        for tr in (session.trace_left, session.trace_right, session.trace_front_left, session.trace_front_right):
            xs += [p[0] for p in tr]
            ys += [p[1] for p in tr]
    else:
        for tr in (session.trace_left, session.trace_right):
            xs += [p[0] for p in tr]
            ys += [p[1] for p in tr]
    if not xs:
        return -pad, pad, -pad, pad
    return (
        min(xs) - pad,
        max(xs) + pad,
        min(ys) - pad,
        max(ys) + pad,
    )


def draw_motion_frame(
    ax: plt.Axes,
    session: MotionSession,
    frame_index: int,
    *,
    draw_robot: bool = True,
    tire_linestyle: str = ":",
    tire_linewidth: float = 1.2,
    center_color: str = "#555555",
    rear_left_color: str = "#1565c0",
    rear_right_color: str = "#c62828",
    front_left_color: str = "#2e7d32",
    front_right_color: str = "#f9a825",
    title: str | None = None,
    axis_limits: tuple[float, float, float, float] | None = None,
) -> None:
    """Draw *session* up to and including *frame_index* (clamped). Used for animation frames."""
    ax.clear()
    if title:
        ax.set_title(title, fontsize=11)
    n = len(session.poses)
    if n == 0:
        return
    k = min(max(0, frame_index) + 1, n)
    poses = session.poses[:k]

    if len(poses) < 1:
        return

    xs = [p.x for p in poses]
    ys = [p.y for p in poses]
    ax.plot(xs, ys, color=center_color, linewidth=1.0, label="CoM path", alpha=0.8)

    if session.kind == KinematicsModel.MECANUM:
        flx = [p[0] for p in session.trace_mecanum_fl[:k]]
        fly = [p[1] for p in session.trace_mecanum_fl[:k]]
        frx = [p[0] for p in session.trace_mecanum_fr[:k]]
        fry = [p[1] for p in session.trace_mecanum_fr[:k]]
        rlx = [p[0] for p in session.trace_mecanum_rl[:k]]
        rly = [p[1] for p in session.trace_mecanum_rl[:k]]
        rrx = [p[0] for p in session.trace_mecanum_rr[:k]]
        rry = [p[1] for p in session.trace_mecanum_rr[:k]]
        ax.plot(flx, fly, color=front_left_color, linestyle=tire_linestyle, linewidth=tire_linewidth, label="Mecanum FL")
        ax.plot(frx, fry, color=front_right_color, linestyle=tire_linestyle, linewidth=tire_linewidth, label="Mecanum FR")
        ax.plot(rlx, rly, color=rear_left_color, linestyle=tire_linestyle, linewidth=tire_linewidth, label="Mecanum RL")
        ax.plot(rrx, rry, color=rear_right_color, linestyle=tire_linestyle, linewidth=tire_linewidth, label="Mecanum RR")
    elif session.kind in (KinematicsModel.ACKERMANN, KinematicsModel.BICYCLE) and session.trace_front_left:
        rlx = [p[0] for p in session.trace_left[:k]]
        rly = [p[1] for p in session.trace_left[:k]]
        rrx = [p[0] for p in session.trace_right[:k]]
        rry = [p[1] for p in session.trace_right[:k]]
        flx = [p[0] for p in session.trace_front_left[:k]]
        fly = [p[1] for p in session.trace_front_left[:k]]
        frx = [p[0] for p in session.trace_front_right[:k]]
        fry = [p[1] for p in session.trace_front_right[:k]]
        ax.plot(rlx, rly, color=rear_left_color, linestyle=tire_linestyle, linewidth=tire_linewidth, label="Rear L")
        ax.plot(rrx, rry, color=rear_right_color, linestyle=tire_linestyle, linewidth=tire_linewidth, label="Rear R")
        ax.plot(flx, fly, color=front_left_color, linestyle=tire_linestyle, linewidth=tire_linewidth, label="Front L")
        ax.plot(frx, fry, color=front_right_color, linestyle=tire_linestyle, linewidth=tire_linewidth, label="Front R")
    else:
        lx = [p[0] for p in session.trace_left[:k]]
        ly = [p[1] for p in session.trace_left[:k]]
        rx = [p[0] for p in session.trace_right[:k]]
        ry = [p[1] for p in session.trace_right[:k]]
        ax.plot(lx, ly, color=rear_left_color, linestyle=tire_linestyle, linewidth=tire_linewidth, label="Left tire")
        ax.plot(rx, ry, color=rear_right_color, linestyle=tire_linestyle, linewidth=tire_linewidth, label="Right tire")

    if draw_robot:
        simulate(poses[-1], ax=ax, show=False)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    if axis_limits is not None:
        xmin, xmax, ymin, ymax = axis_limits
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
    else:
        pad = 0.8
        all_x = xs
        all_y = ys
        if session.kind == KinematicsModel.MECANUM:
            for tr in (
                session.trace_mecanum_fl,
                session.trace_mecanum_fr,
                session.trace_mecanum_rl,
                session.trace_mecanum_rr,
            ):
                all_x += [p[0] for p in tr[:k]]
                all_y += [p[1] for p in tr[:k]]
        elif session.kind in (KinematicsModel.ACKERMANN, KinematicsModel.BICYCLE) and session.trace_front_left:
            for tr in (session.trace_left, session.trace_right, session.trace_front_left, session.trace_front_right):
                all_x += [p[0] for p in tr[:k]]
                all_y += [p[1] for p in tr[:k]]
        else:
            all_x += [p[0] for p in session.trace_left[:k]]
            all_x += [p[0] for p in session.trace_right[:k]]
            all_y += [p[1] for p in session.trace_left[:k]]
            all_y += [p[1] for p in session.trace_right[:k]]
        ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
        ax.set_ylim(min(all_y) - pad, max(all_y) + pad)


def plot_motion(
    session: MotionSession,
    *,
    ax: plt.Axes | None = None,
    show: bool = True,
    draw_robot: bool = True,
    tire_linestyle: str = ":",
    tire_linewidth: float = 1.2,
    center_color: str = "#555555",
    rear_left_color: str = "#1565c0",
    rear_right_color: str = "#c62828",
    front_left_color: str = "#2e7d32",
    front_right_color: str = "#f9a825",
) -> plt.Axes:
    """Plot full pose path and dotted tire traces from a :class:`MotionSession`."""
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 8))

    n = len(session.poses)
    if n < 2:
        if draw_robot:
            simulate(session.pose, ax=ax, show=False)
        ax.set_aspect("equal", adjustable="box")
        if show:
            plt.show()
        return ax

    draw_motion_frame(
        ax,
        session,
        n - 1,
        draw_robot=draw_robot,
        tire_linestyle=tire_linestyle,
        tire_linewidth=tire_linewidth,
        center_color=center_color,
        rear_left_color=rear_left_color,
        rear_right_color=rear_right_color,
        front_left_color=front_left_color,
        front_right_color=front_right_color,
        axis_limits=motion_bounds(session),
    )

    if show:
        plt.show()
    return ax
