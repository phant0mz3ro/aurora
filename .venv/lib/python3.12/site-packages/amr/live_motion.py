from __future__ import annotations

import math
from typing import TYPE_CHECKING, TextIO

import matplotlib.animation as mplanimation
import matplotlib.pyplot as plt

from amr.kinematics.params import (
    AckermannParams,
    DifferentialParams,
    KinematicsModel,
    MecanumParams,
    UnicycleParams,
)
from amr.motion import MotionSession
from amr.playback_log import (
    PlaybackLogOptions,
    effective_interval_ms,
    log_frame,
    log_loop_restart,
    log_segment_params,
    log_session_start,
)
from amr.plot_motion import draw_motion_frame, motion_bounds
from amr.pose import pose

if TYPE_CHECKING:
    from matplotlib.animation import FuncAnimation


def play_motion(
    session: MotionSession,
    *,
    interval_ms: int = 25,
    playback_speed: float = 1.0,
    repeat: bool = True,
    title: str | None = None,
    figsize: tuple[float, float] = (8.0, 8.0),
    show: bool = True,
    log: bool = False,
    log_every_n_frames: int = 12,
    log_detailed: bool = False,
    log_file: TextIO | None = None,
    log_options: PlaybackLogOptions | None = None,
) -> FuncAnimation | None:
    """Animate a :class:`MotionSession` so paths and robot update in real time.

    Parameters
    ----------
    interval_ms
        Base delay between frames (milliseconds). Effective delay is divided by
        ``playback_speed``.
    playback_speed
        Values above ``1.0`` play faster (shorter interval); below ``1.0`` play slower.
    log
        When True, print formatted teaching notes to the terminal (or ``log_file``).
    log_every_n_frames
        Throttle frame logs (always logs frame 0).
    log_detailed
        Multi-line blocks per log step instead of one compact line.
    log_options
        If provided, overrides ``log``, ``log_every_n_frames``, ``log_detailed``, and ``log_file``.
    """
    eff_ms = effective_interval_ms(float(interval_ms), playback_speed)

    opts = log_options or PlaybackLogOptions(
        enabled=log,
        every_n_frames=max(1, log_every_n_frames),
        detailed_block=log_detailed,
        file=log_file,
    )

    n = len(session.poses)
    if n < 2:
        fig, ax = plt.subplots(figsize=figsize)
        draw_motion_frame(ax, session, max(0, n - 1), title=title, axis_limits=motion_bounds(session))
        if opts.enabled:
            log_session_start(
                session,
                title=title or "Motion playback",
                interval_ms=float(interval_ms),
                playback_speed=playback_speed,
                file=opts.file,
            )
            log_frame(session, max(0, n - 1), options=opts)
        if show:
            plt.show()
        return None

    limits = motion_bounds(session)
    fig, ax = plt.subplots(figsize=figsize)

    if opts.enabled:
        log_session_start(
            session,
            title=title or "Motion playback",
            interval_ms=float(interval_ms),
            playback_speed=playback_speed,
            file=opts.file,
        )

    state = {"prev": -1}

    def _update(frame: int) -> list:
        draw_motion_frame(
            ax,
            session,
            frame,
            title=title,
            axis_limits=limits,
        )
        if opts.enabled:
            if repeat and frame == 0 and state["prev"] == n - 1:
                log_loop_restart(file=opts.file)
            if opts.should_log(frame):
                log_frame(session, frame, options=opts)
            state["prev"] = frame
        return [ax]

    anim = mplanimation.FuncAnimation(
        fig,
        _update,
        frames=n,
        interval=eff_ms,
        repeat=repeat,
        blit=False,
    )
    if show:
        plt.show()
    return anim


def _demo_two_wheel() -> MotionSession:
    s = MotionSession.create(
        pose(0, 0, 0),
        KinematicsModel.TWO_WHEEL,
        dt=0.02,
        unicycle=UnicycleParams(track_width=0.42),
    )
    s.forward(1.2, 0.5)
    s.turn_left(math.pi / 2, 0.9)
    s.forward(0.8, 0.5)
    return s


def _demo_differential() -> MotionSession:
    s = MotionSession.create(
        pose(0, 0, 0),
        KinematicsModel.DIFFERENTIAL,
        dt=0.015,
        differential=DifferentialParams(track_width=0.4, max_wheel_speed=2.0),
    )
    s.forward_wheels(1.0, 0.9)
    s.turn_right(math.pi / 3, 1.0)
    s.differential_drive_wheels(0.4, 1.1, duration=1.2)
    return s


def _demo_ackermann() -> MotionSession:
    s = MotionSession.create(
        pose(0, 0, 0),
        KinematicsModel.ACKERMANN,
        dt=0.02,
        ackermann=AckermannParams(wheelbase=0.55, track_width=0.36, max_steering_angle=0.48, max_speed=1.0),
    )
    s.forward(1.5, 0.55)
    s.turn_left(math.radians(40), 0.75)
    s.forward(1.0, 0.45)
    return s


def _demo_mecanum() -> MotionSession:
    s = MotionSession.create(
        pose(0, 0, 0),
        KinematicsModel.MECANUM,
        dt=0.015,
        mecanum=MecanumParams(half_length_y=0.26, half_width_x=0.2, max_wheel_speed=2.5),
    )
    s.forward(0.7, 0.6)
    s.strafe_right(0.6, 0.55)
    s.turn_left(math.pi / 4, 1.0)
    s.mecanum_drive_wheels(0.4, 0.4, 0.5, 0.5, duration=0.8)
    return s


def play_all_kinematics_live(
    *,
    interval_ms: int = 28,
    playback_speed: float = 1.0,
    pause_frames: int = 25,
    repeat: bool = True,
    show: bool = True,
    log: bool = False,
    log_every_n_frames: int = 10,
    log_detailed: bool = False,
    log_file: TextIO | None = None,
    log_options: PlaybackLogOptions | None = None,
) -> FuncAnimation | None:
    """Play short demos for two-wheel → differential → Ackermann → mecanum in one window."""
    eff_ms = effective_interval_ms(float(interval_ms), playback_speed)
    opts = log_options or PlaybackLogOptions(
        enabled=log,
        every_n_frames=max(1, log_every_n_frames),
        detailed_block=log_detailed,
        file=log_file,
    )

    segments: list[tuple[str, MotionSession]] = [
        ("Two-wheel", _demo_two_wheel()),
        ("Differential drive", _demo_differential()),
        ("Ackermann (4-wheel)", _demo_ackermann()),
        ("Mecanum", _demo_mecanum()),
    ]
    lengths = [len(s[1].poses) for s in segments]
    total_frames = sum(ln + pause_frames for ln in lengths)
    bounds_list = [motion_bounds(s[1]) for s in segments]
    xmin = min(b[0] for b in bounds_list)
    xmax = max(b[1] for b in bounds_list)
    ymin = min(b[2] for b in bounds_list)
    ymax = max(b[3] for b in bounds_list)
    limits = (xmin, xmax, ymin, ymax)

    fig, ax = plt.subplots(figsize=(8, 8))

    if opts.enabled:
        out = opts.file
        print("\n", file=out)
        print("=" * 72, file=out)
        print("  AuroraMR — multi-kinematics tour (see docstrings in amr.live_motion)", file=out)
        print("=" * 72, file=out)
        print(
            f"  Base interval: {interval_ms} ms  |  playback_speed: {playback_speed:.2f}×  "
            f"→ effective {eff_ms:.1f} ms/frame",
            file=out,
        )
        print("", file=out)

    log_seg = {"idx": -1}

    def _global_frame(i: int) -> tuple[int, int]:
        t = 0
        for seg_idx, ln in enumerate(lengths):
            block = ln + pause_frames
            if i < t + block:
                local = i - t
                if local < ln:
                    return seg_idx, local
                return seg_idx, ln - 1
            t += block
        return len(segments) - 1, lengths[-1] - 1

    state = {"prev_global": -1, "loop": 0}

    def _update(i: int) -> list:
        seg_i, fi = _global_frame(i)
        name, sess = segments[seg_i]
        if opts.enabled and seg_i != log_seg["idx"]:
            log_segment_params(sess, name, file=opts.file)
            log_seg["idx"] = seg_i
        draw_motion_frame(ax, sess, fi, title=f"{name} (live demo)", axis_limits=limits)
        if opts.enabled:
            if repeat and i == 0 and state["prev_global"] == total_frames - 1:
                log_loop_restart(file=opts.file)
                log_seg["idx"] = -1
                state["loop"] += 1
            if opts.should_log(fi):
                log_frame(sess, fi, options=opts)
            state["prev_global"] = i
        return [ax]

    anim = mplanimation.FuncAnimation(
        fig,
        _update,
        frames=total_frames,
        interval=eff_ms,
        repeat=repeat,
        blit=False,
    )
    if show:
        plt.show()
    return anim


def play_motion_by_kind(
    kind: str,
    *,
    interval_ms: int = 28,
    playback_speed: float = 1.0,
    show: bool = True,
    log: bool = False,
    log_every_n_frames: int = 12,
    log_detailed: bool = False,
    log_file: TextIO | None = None,
    log_options: PlaybackLogOptions | None = None,
) -> FuncAnimation | None:
    """Play one built-in demo: ``two_wheel`` (or legacy ``unicycle``), ``differential``, ``ackermann``, ``mecanum``."""
    builders = {
        "two_wheel": _demo_two_wheel,
        "unicycle": _demo_two_wheel,
        "differential": _demo_differential,
        "ackermann": _demo_ackermann,
        "mecanum": _demo_mecanum,
    }
    k = kind.lower().strip()
    if k not in builders:
        raise ValueError(f"kind must be one of {sorted(builders)}")
    session = builders[k]()
    title = f"{k.replace('_', ' ').title()} (live demo)"
    return play_motion(
        session,
        interval_ms=interval_ms,
        playback_speed=playback_speed,
        title=title,
        show=show,
        log=log,
        log_every_n_frames=log_every_n_frames,
        log_detailed=log_detailed,
        log_file=log_file,
        log_options=log_options,
    )
