"""Formatted terminal output for live motion playback (student / lab use)."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Any, TextIO

from amr.kinematics.params import KinematicsModel
from amr.motion import MotionSession
from amr.kinematics.integration import wrap_pi


def _rule(char: str = "=", width: int = 72) -> str:
    return char * width


def _heading_cardinal(theta_rad: float) -> str:
    """Rough compass label for θ (CCW from north)."""
    d = math.degrees(wrap_pi(theta_rad)) % 360
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    idx = int((d + 22.5) // 45) % 8
    return dirs[idx]


def effective_interval_ms(base_interval_ms: float, playback_speed: float) -> float:
    """Faster playback (*playback_speed* > 1) uses shorter interval between frames."""
    if playback_speed <= 0:
        raise ValueError("playback_speed must be positive")
    return max(1.0, base_interval_ms / playback_speed)


def format_kinematics_params(session: MotionSession) -> str:
    """Human-readable summary of the active kinematics parameters."""
    lines: list[str] = []
    lines.append("  World frame:       x = right, y = up; heading θ is CCW from north (+y).")
    k = session.kind
    lines.append(f"  Model kind:        {k.value}")
    lines.append(f"  Integration dt:    {session.dt:.6f} s  (simulation time step)")
    if session.unicycle is not None:
        u = session.unicycle
        lines.append(f"  Two-wheel:         track_width={u.track_width:.2f} m, v_max={u.max_linear_speed:.2f} m/s, ω_max={u.max_angular_speed:.2f} rad/s")
    if session.differential is not None:
        d = session.differential
        lines.append(f"  Differential:      track width={d.track_width:.2f} m, r_wheel={d.wheel_radius:.2f} m, wheel_speed_max={d.max_wheel_speed:.2f} m/s")
    if session.ackermann is not None:
        a = session.ackermann
        lines.append(
            f"  Ackermann:         wheelbase={a.wheelbase:.2f} m, track={a.track_width:.2f} m, "
            f"|δ|_max={math.degrees(a.max_steering_angle):.1f}°, v_max={a.max_speed:.2f} m/s"
        )
    if session.mecanum is not None:
        m = session.mecanum
        lines.append(
            f"  Mecanum:           half_length_y={m.half_length_y:.2f} m, half_width_x={m.half_width_x:.2f} m, "
            f"|v|_wheel_max={m.max_wheel_speed:.2f} m/s"
        )
    return "\n".join(lines)


def _estimate_twist(session: MotionSession, i: int) -> tuple[float | None, float | None]:
    """Approximate forward speed and yaw rate from successive poses."""
    if i < 1 or i >= len(session.poses):
        return None, None
    p0, p1 = session.poses[i - 1], session.poses[i]
    dt = session.dt
    if dt < 1e-12:
        return None, None
    dx = p1.x - p0.x
    dy = p1.y - p0.y
    d = math.hypot(dx, dy)
    # Forward direction (-sin θ, cos θ) at midpoint
    th = 0.5 * (p0.theta + p1.theta)
    fx, fy = -math.sin(th), math.cos(th)
    v = (dx * fx + dy * fy) / dt
    omega = wrap_pi(p1.theta - p0.theta) / dt
    return v, omega


def format_frame_line(
    session: MotionSession,
    frame_index: int,
    *,
    include_velocity: bool = True,
    include_tires: bool = False,
) -> str:
    """One compact line for a single frame."""
    p = session.poses[frame_index]
    t_sim = frame_index * session.dt
    th_deg = math.degrees(wrap_pi(p.theta))
    card = _heading_cardinal(p.theta)
    parts = [
        f"frame={frame_index:5d}",
        f"t_sim={t_sim:7.3f}s",
        f"x={p.x:7.3f}",
        f"y={p.y:7.3f}",
        f"θ={th_deg:7.2f}°",
        f"({card})",
    ]
    if include_velocity and frame_index > 0:
        v, omega = _estimate_twist(session, frame_index)
        if v is not None:
            parts.append(f"v≈{v:6.3f}m/s")
            parts.append(f"ω≈{omega:6.3f}rad/s")
    if include_tires and frame_index < len(session.trace_left):
        lx, ly = session.trace_left[frame_index]
        parts.append(f"rear_L=({lx:.2f},{ly:.2f})")
    return "  " + "  ".join(parts)


def format_frame_block(
    session: MotionSession,
    frame_index: int,
    *,
    include_velocity: bool = True,
) -> str:
    """Multi-line block for teaching (more detail than one line)."""
    p = session.poses[frame_index]
    t_sim = frame_index * session.dt
    th = wrap_pi(p.theta)
    lines = [
        f"  Simulation time:   {t_sim:.4f} s   (frame index × dt)",
        f"  Pose (world):      x = {p.x:.5f} m",
        f"                     y = {p.y:.5f} m",
        f"  Heading θ:         {math.degrees(th):.4f}°  ({th:.6f} rad, CCW from +y / north)",
        f"                     Compass sector: ~{_heading_cardinal(p.theta)}",
    ]
    if include_velocity and frame_index > 0:
        v, omega = _estimate_twist(session, frame_index)
        if v is not None:
            lines.append(f"  Est. twist:        v ≈ {v:.5f} m/s (forward),  ω ≈ {omega:.5f} rad/s (CCW)")
            lines.append("                     (finite difference between consecutive poses / dt)")
    if frame_index < len(session.trace_left):
        lx, ly = session.trace_left[frame_index]
        rx, ry = session.trace_right[frame_index]
        lines.append(f"  Rear contacts:     L ({lx:.4f}, {ly:.4f})   R ({rx:.4f}, {ry:.4f})")
    if session.kind == KinematicsModel.MECANUM and frame_index < len(session.trace_mecanum_fl):
        fl = session.trace_mecanum_fl[frame_index]
        fr = session.trace_mecanum_fr[frame_index]
        lines.append(f"  Mecanum FL/FR:     ({fl[0]:.4f}, {fl[1]:.4f})   ({fr[0]:.4f}, {fr[1]:.4f})")
    if session.kind in (KinematicsModel.ACKERMANN, KinematicsModel.BICYCLE) and frame_index < len(session.trace_front_left):
        fl = session.trace_front_left[frame_index]
        fr = session.trace_front_right[frame_index]
        lines.append(f"  Front corners:     L ({fl[0]:.4f}, {fl[1]:.4f})   R ({fr[0]:.4f}, {fr[1]:.4f})")
    return "\n".join(lines)


def log_session_start(
    session: MotionSession,
    *,
    title: str | None = None,
    interval_ms: float | None = None,
    playback_speed: float | None = None,
    file: TextIO | None = None,
) -> None:
    """Print a summary banner when playback begins."""
    out = file or sys.stdout
    name = title or "Motion playback"
    lines = [
        "",
        _rule("="),
        f"  {name}",
        _rule("="),
        f"  Total poses:       {len(session.poses)}",
        format_kinematics_params(session),
    ]
    if interval_ms is not None and playback_speed is not None:
        eff = effective_interval_ms(interval_ms, playback_speed)
        lines.append(
            f"  Playback:          base interval = {interval_ms:.0f} ms × speed {playback_speed:.2f}× "
            f"→ effective {eff:.1f} ms/frame (~{1000.0 / eff:.1f} fps display)"
        )
    elif interval_ms is not None:
        lines.append(f"  Playback:          {interval_ms:.0f} ms per frame (~{1000.0 / interval_ms:.1f} fps)")
    lines.append(_rule("="))
    lines.append("")
    print("\n".join(lines), file=out)


def log_segment_banner(segment_name: str, *, file: TextIO | None = None) -> None:
    out = file or sys.stdout
    print(
        f"\n{_rule('-')}\n  ▶ Segment: {segment_name}\n{_rule('-')}\n",
        file=out,
    )


def log_segment_params(session: MotionSession, segment_name: str, *, file: TextIO | None = None) -> None:
    """After a segment banner, print kinematics parameters for that session."""
    out = file or sys.stdout
    log_segment_banner(segment_name, file=out)
    print(format_kinematics_params(session), file=out)
    print(file=out)


def log_loop_restart(*, file: TextIO | None = None) -> None:
    out = file or sys.stdout
    print(f"\n  ⟲ Animation loop restarted\n{_rule('.')}\n", file=out)


@dataclass
class PlaybackLogOptions:
    """Terminal logging while an animation runs (for teaching / debugging)."""

    enabled: bool = False
    every_n_frames: int = 12
    detailed_block: bool = False
    include_velocity: bool = True
    include_tire_sample: bool = False
    file: Any = None  # TextIO or tee with write/flush

    def should_log(self, frame: int) -> bool:
        if not self.enabled:
            return False
        return frame == 0 or (frame % max(1, self.every_n_frames) == 0)


def log_frame(
    session: MotionSession,
    frame_index: int,
    *,
    options: PlaybackLogOptions,
) -> None:
    out = options.file or sys.stdout
    if options.detailed_block:
        print(format_frame_block(session, frame_index, include_velocity=options.include_velocity), file=out)
        print(_rule(".", width=60), file=out)
    else:
        print(
            format_frame_line(
                session,
                frame_index,
                include_velocity=options.include_velocity,
                include_tires=options.include_tire_sample,
            ),
            file=out,
        )
