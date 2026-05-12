from __future__ import annotations

import math

import numpy as np

from amr.pose import Pose, pose


def wrap_pi(angle: float) -> float:
    """Wrap angle to ``(-π, π]``."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def integrate_unicycle(p: Pose, v: float, omega: float, dt: float) -> Pose:
    """Planar two-wheel / unicycle kinematics: forward speed *v*, yaw rate *omega* (CCW), *dt* in seconds."""
    x2 = p.x - v * math.sin(p.theta) * dt
    y2 = p.y + v * math.cos(p.theta) * dt
    t2 = wrap_pi(p.theta + omega * dt)
    return pose(x2, y2, t2)


def differential_to_twist(v_left: float, v_right: float, track_width: float) -> tuple[float, float]:
    """Linear speed at center and yaw rate from left/right wheel speeds (m/s)."""
    v = 0.5 * (v_right + v_left)
    omega = (v_right - v_left) / track_width
    return v, omega


def integrate_bicycle(p: Pose, v: float, steering_angle: float, wheelbase: float, dt: float) -> Pose:
    """Rear-axle bicycle / Ackermann base model: *v* at rear, equivalent front steer."""
    omega = v * math.tan(steering_angle) / wheelbase
    return integrate_unicycle(p, v, omega, dt)


def ackermann_front_steering_angles(
    delta_center: float,
    wheelbase: float,
    track_width: float,
) -> tuple[float, float]:
    """Ideal Ackermann front steer angles (left, right) in radians for center steer *delta_center*."""
    if abs(delta_center) < 1e-9:
        return 0.0, 0.0
    tan_d = math.tan(delta_center)
    if abs(tan_d) < 1e-14:
        return 0.0, 0.0
    r = wheelbase / tan_d
    w = track_width
    ri = r - w / 2
    ro = r + w / 2
    if abs(ri) < 1e-6 or abs(ro) < 1e-6:
        return delta_center, delta_center
    if r > 0:
        d_left = math.atan(wheelbase / ri)
        d_right = math.atan(wheelbase / ro)
    else:
        d_left = math.atan(wheelbase / ro)
        d_right = math.atan(wheelbase / ri)
    return d_left, d_right


def tire_offsets_body(track_width: float) -> tuple[np.ndarray, np.ndarray]:
    """Left / right contact offsets in body frame (+*y* forward, +*x* right)."""
    h = 0.5 * track_width
    left = np.array([-h, 0.0], dtype=float)
    right = np.array([h, 0.0], dtype=float)
    return left, right


def body_to_world_jacobian(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def wheel_contacts_world(
    center_xy: tuple[float, float],
    theta: float,
    track_width: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """World positions of left and right wheel contacts (rear-axle centered slice)."""
    x, y = center_xy
    left_b, right_b = tire_offsets_body(track_width)
    r = body_to_world_jacobian(theta)
    lw = r @ left_b
    rw = r @ right_b
    return (float(x + lw[0]), float(y + lw[1])), (float(x + rw[0]), float(y + rw[1]))


def four_wheel_contacts_rear_axle(
    rear_xy: tuple[float, float],
    theta: float,
    wheelbase: float,
    track_width: float,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float]]:
    """RL, RR, FL, FR contact points (body +y forward, rear axle at origin)."""
    w = track_width
    l = wheelbase
    corners = [(-w / 2, 0.0), (w / 2, 0.0), (-w / 2, l), (w / 2, l)]
    r = body_to_world_jacobian(theta)
    x0, y0 = rear_xy
    out: list[tuple[float, float]] = []
    for px, py in corners:
        wx = x0 + r[0, 0] * px + r[0, 1] * py
        wy = y0 + r[1, 0] * px + r[1, 1] * py
        out.append((float(wx), float(wy)))
    return out[0], out[1], out[2], out[3]


def front_axle_center_world(
    center_xy: tuple[float, float],
    theta: float,
    wheelbase: float,
) -> tuple[float, float]:
    """Front axle midpoint in world frame (rear reference *center_xy*)."""
    x, y = center_xy
    fx = -math.sin(theta)
    fy = math.cos(theta)
    return (x + wheelbase * fx, y + wheelbase * fy)


def mecanum_twist(
    v_fl: float,
    v_fr: float,
    v_rl: float,
    v_rr: float,
    half_length_y: float,
    half_width_x: float,
) -> tuple[float, float, float]:
    """Body-frame lateral *v_x* (right), forward *v_y*, yaw rate *omega* (CCW), from wheel linear speeds."""
    v_y = 0.25 * (v_fl + v_fr + v_rl + v_rr)
    v_x = 0.25 * (-v_fl + v_fr + v_rl - v_rr)
    k = 4.0 * (half_length_y + half_width_x)
    if k < 1e-12:
        omega = 0.0
    else:
        omega = (-v_fl + v_fr - v_rl + v_rr) / k
    return v_x, v_y, omega


def integrate_mecanum(
    p: Pose,
    v_fl: float,
    v_fr: float,
    v_rl: float,
    v_rr: float,
    half_length_y: float,
    half_width_x: float,
    dt: float,
) -> Pose:
    """Integrate pose at robot geometric center for X-pattern mecanum."""
    vx_b, vy_b, omega = mecanum_twist(v_fl, v_fr, v_rl, v_rr, half_length_y, half_width_x)
    forward = vy_b
    right = vx_b
    vx_w = forward * (-math.sin(p.theta)) + right * math.cos(p.theta)
    vy_w = forward * math.cos(p.theta) + right * math.sin(p.theta)
    x2 = p.x + vx_w * dt
    y2 = p.y + vy_w * dt
    t2 = wrap_pi(p.theta + omega * dt)
    return pose(x2, y2, t2)


def mecanum_wheel_contacts_world(
    center_xy: tuple[float, float],
    theta: float,
    half_width_x: float,
    half_length_y: float,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float]]:
    """FL, FR, RL, RR in world frame (+y forward in body)."""
    corners = [
        (-half_width_x, half_length_y),
        (half_width_x, half_length_y),
        (-half_width_x, -half_length_y),
        (half_width_x, -half_length_y),
    ]
    r = body_to_world_jacobian(theta)
    x0, y0 = center_xy
    out: list[tuple[float, float]] = []
    for px, py in corners:
        wx = x0 + r[0, 0] * px + r[0, 1] * py
        wy = y0 + r[1, 0] * px + r[1, 1] * py
        out.append((float(wx), float(wy)))
    return out[0], out[1], out[2], out[3]
