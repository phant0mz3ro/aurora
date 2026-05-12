from __future__ import annotations

import math
from dataclasses import dataclass, field

from amr.kinematics.integration import (
    differential_to_twist,
    four_wheel_contacts_rear_axle,
    front_axle_center_world,
    integrate_bicycle,
    integrate_mecanum,
    integrate_unicycle,
    mecanum_wheel_contacts_world,
    wheel_contacts_world,
    wrap_pi,
)
from amr.kinematics.params import (
    AckermannParams,
    BicycleParams,
    DifferentialParams,
    KinematicsModel,
    MecanumParams,
    UnicycleParams,
)
from amr.pose import Pose


def _heading_to_point(dx: float, dy: float) -> float:
    """Heading (CCW from north) for motion toward relative vector *(dx, dy)*."""
    return math.atan2(-dx, dy)


def _is_ackermann(kind: KinematicsModel) -> bool:
    return kind in (KinematicsModel.ACKERMANN, KinematicsModel.BICYCLE)


@dataclass
class MotionSession:
    """Simulated motion with recorded poses and tire contact traces."""

    pose: Pose
    kind: KinematicsModel
    dt: float
    unicycle: UnicycleParams | None = None
    differential: DifferentialParams | None = None
    ackermann: AckermannParams | None = None
    mecanum: MecanumParams | None = None
    bicycle: BicycleParams | None = None
    poses: list[Pose] = field(default_factory=list)
    trace_left: list[tuple[float, float]] = field(default_factory=list)
    trace_right: list[tuple[float, float]] = field(default_factory=list)
    trace_front_left: list[tuple[float, float]] = field(default_factory=list)
    trace_front_right: list[tuple[float, float]] = field(default_factory=list)
    trace_mecanum_fl: list[tuple[float, float]] = field(default_factory=list)
    trace_mecanum_fr: list[tuple[float, float]] = field(default_factory=list)
    trace_mecanum_rl: list[tuple[float, float]] = field(default_factory=list)
    trace_mecanum_rr: list[tuple[float, float]] = field(default_factory=list)
    trace_front: list[tuple[float, float]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.kind == KinematicsModel.TWO_WHEEL and self.unicycle is None:
            self.unicycle = UnicycleParams()
        if self.kind == KinematicsModel.DIFFERENTIAL and self.differential is None:
            self.differential = DifferentialParams()
        if _is_ackermann(self.kind):
            if self.ackermann is None and self.bicycle is not None:
                self.ackermann = self.bicycle.to_ackermann()
            elif self.ackermann is None:
                self.ackermann = AckermannParams()
        if self.kind == KinematicsModel.MECANUM and self.mecanum is None:
            self.mecanum = MecanumParams()
        self._append_state()

    @classmethod
    def create(
        cls,
        initial: Pose,
        kind: KinematicsModel,
        *,
        dt: float = 0.02,
        unicycle: UnicycleParams | None = None,
        differential: DifferentialParams | None = None,
        ackermann: AckermannParams | None = None,
        mecanum: MecanumParams | None = None,
        bicycle: BicycleParams | None = None,
    ) -> MotionSession:
        return cls(
            pose=initial,
            kind=kind,
            dt=dt,
            unicycle=unicycle,
            differential=differential,
            ackermann=ackermann,
            mecanum=mecanum,
            bicycle=bicycle,
        )

    def _track(self) -> float:
        if self.kind == KinematicsModel.TWO_WHEEL:
            assert self.unicycle is not None
            return self.unicycle.track_width
        if self.kind == KinematicsModel.DIFFERENTIAL:
            assert self.differential is not None
            return self.differential.track_width
        if _is_ackermann(self.kind):
            assert self.ackermann is not None
            return self.ackermann.track_width
        assert self.mecanum is not None
        return 2.0 * self.mecanum.half_width_x

    def _append_state(self) -> None:
        self.poses.append(self.pose)
        if self.kind == KinematicsModel.MECANUM:
            assert self.mecanum is not None
            m = self.mecanum
            fl, fr, rl, rr = mecanum_wheel_contacts_world(
                (self.pose.x, self.pose.y),
                self.pose.theta,
                m.half_width_x,
                m.half_length_y,
            )
            self.trace_mecanum_fl.append(fl)
            self.trace_mecanum_fr.append(fr)
            self.trace_mecanum_rl.append(rl)
            self.trace_mecanum_rr.append(rr)
            self.trace_left.append(rl)
            self.trace_right.append(rr)
            return
        if _is_ackermann(self.kind):
            assert self.ackermann is not None
            a = self.ackermann
            rl, rr, fl, fr = four_wheel_contacts_rear_axle(
                (self.pose.x, self.pose.y),
                self.pose.theta,
                a.wheelbase,
                a.track_width,
            )
            self.trace_left.append(rl)
            self.trace_right.append(rr)
            self.trace_front_left.append(fl)
            self.trace_front_right.append(fr)
            self.trace_front.append(
                front_axle_center_world((self.pose.x, self.pose.y), self.pose.theta, a.wheelbase)
            )
            return
        tw = self._track()
        left, right = wheel_contacts_world((self.pose.x, self.pose.y), self.pose.theta, tw)
        self.trace_left.append(left)
        self.trace_right.append(right)

    def _step_unicycle_dt(self, v: float, omega: float, dt: float) -> None:
        assert self.kind == KinematicsModel.TWO_WHEEL and self.unicycle is not None
        u = self.unicycle
        v = max(-u.max_linear_speed, min(u.max_linear_speed, v))
        omega = max(-u.max_angular_speed, min(u.max_angular_speed, omega))
        self.pose = integrate_unicycle(self.pose, v, omega, dt)
        self._append_state()

    def _step_differential_wheels_dt(self, v_left: float, v_right: float, dt: float) -> None:
        assert self.kind == KinematicsModel.DIFFERENTIAL and self.differential is not None
        w = self.differential
        v_left = max(-w.max_wheel_speed, min(w.max_wheel_speed, v_left))
        v_right = max(-w.max_wheel_speed, min(w.max_wheel_speed, v_right))
        v, omega = differential_to_twist(v_left, v_right, w.track_width)
        self.pose = integrate_unicycle(self.pose, v, omega, dt)
        self._append_state()

    def _step_ackermann_dt(self, v: float, steering_angle: float, dt: float) -> None:
        assert _is_ackermann(self.kind) and self.ackermann is not None
        b = self.ackermann
        v = max(-b.max_speed, min(b.max_speed, v))
        steering_angle = max(-b.max_steering_angle, min(b.max_steering_angle, steering_angle))
        self.pose = integrate_bicycle(self.pose, v, steering_angle, b.wheelbase, dt)
        self._append_state()

    def _step_mecanum_dt(self, v_fl: float, v_fr: float, v_rl: float, v_rr: float, dt: float) -> None:
        assert self.kind == KinematicsModel.MECANUM and self.mecanum is not None
        m = self.mecanum
        mx = m.max_wheel_speed
        v_fl = max(-mx, min(mx, v_fl))
        v_fr = max(-mx, min(mx, v_fr))
        v_rl = max(-mx, min(mx, v_rl))
        v_rr = max(-mx, min(mx, v_rr))
        self.pose = integrate_mecanum(
            self.pose, v_fl, v_fr, v_rl, v_rr, m.half_length_y, m.half_width_x, dt
        )
        self._append_state()

    def forward(self, distance: float, speed: float) -> None:
        """Drive forward along current heading by *distance* (m) at linear speed *speed* (>0)."""
        if _is_ackermann(self.kind):
            self._ackermann_along(distance, abs(speed))
            return
        if self.kind == KinematicsModel.MECANUM:
            self._mecanum_forward(distance, abs(speed))
            return
        self._drive_straight(distance, abs(speed))

    def backward(self, distance: float, speed: float) -> None:
        """Move backward by *distance* at positive *speed* magnitude."""
        if _is_ackermann(self.kind):
            self._ackermann_along(-distance, abs(speed))
            return
        if self.kind == KinematicsModel.MECANUM:
            self._mecanum_forward(-distance, abs(speed))
            return
        self._drive_straight(-distance, abs(speed))

    def _drive_straight(self, signed_distance: float, speed_mag: float) -> None:
        if speed_mag < 1e-12:
            return
        remaining = abs(signed_distance)
        sign = 1.0 if signed_distance >= 0 else -1.0
        v_lin = sign * speed_mag
        while remaining > 1e-7:
            dt_eff = min(self.dt, remaining / speed_mag)
            if self.kind == KinematicsModel.DIFFERENTIAL:
                self._step_differential_wheels_dt(v_lin, v_lin, dt_eff)
            else:
                self._step_unicycle_dt(v_lin, 0.0, dt_eff)
            remaining -= speed_mag * dt_eff

    def _ackermann_along(self, distance: float, speed: float) -> None:
        speed = abs(speed)
        if speed < 1e-12:
            return
        remaining = abs(distance)
        v = math.copysign(speed, distance)
        while remaining > 1e-7:
            dt_eff = min(self.dt, remaining / speed)
            self._step_ackermann_dt(v, 0.0, dt_eff)
            remaining -= speed * dt_eff

    def _mecanum_forward(self, signed_distance: float, speed_mag: float) -> None:
        if speed_mag < 1e-12:
            return
        remaining = abs(signed_distance)
        v = math.copysign(speed_mag, signed_distance)
        while remaining > 1e-7:
            dt_eff = min(self.dt, remaining / speed_mag)
            self._step_mecanum_dt(v, v, v, v, dt_eff)
            remaining -= speed_mag * dt_eff

    def strafe_right(self, distance: float, speed: float) -> None:
        """Mecanum: strafe along body +x (right) by *distance* (m) at wheel-speed scale *speed*."""
        if self.kind != KinematicsModel.MECANUM:
            raise TypeError("strafe_right is only for MECANUM")
        speed = abs(speed)
        if speed < 1e-12:
            return
        remaining = abs(distance)
        sgn = 1.0 if distance >= 0 else -1.0
        while remaining > 1e-7:
            dt_eff = min(self.dt, remaining / speed)
            u = sgn * speed
            self._step_mecanum_dt(-u, u, u, -u, dt_eff)
            remaining -= speed * dt_eff

    def turn_left(self, angle_rad: float, angular_speed: float) -> None:
        """Turn CCW (left) by *angle_rad* at angular speed magnitude (rad/s)."""
        if _is_ackermann(self.kind):
            self._ackermann_turn_arc(abs(angle_rad), angular_speed, left=True)
            return
        if self.kind == KinematicsModel.MECANUM:
            self._mecanum_turn_in_place(abs(angle_rad), abs(angular_speed), ccw=True)
            return
        self._turn_in_place(abs(angle_rad), angular_speed, ccw=True)

    def turn_right(self, angle_rad: float, angular_speed: float) -> None:
        """Turn CW (right) by *angle_rad* at angular speed magnitude (rad/s)."""
        if _is_ackermann(self.kind):
            self._ackermann_turn_arc(abs(angle_rad), angular_speed, left=False)
            return
        if self.kind == KinematicsModel.MECANUM:
            self._mecanum_turn_in_place(abs(angle_rad), abs(angular_speed), ccw=False)
            return
        self._turn_in_place(abs(angle_rad), angular_speed, ccw=False)

    def _ackermann_turn_arc(self, angle_rad: float, angular_speed: float, *, left: bool) -> None:
        assert self.ackermann is not None
        target = wrap_pi(self.pose.theta + (angle_rad if left else -angle_rad))
        v_mag = max(0.15 * self.ackermann.max_speed, 1e-3)
        delta = self.ackermann.max_steering_angle if left else -self.ackermann.max_steering_angle
        guard = 0
        while abs(wrap_pi(target - self.pose.theta)) > 0.02 and guard < 200_000:
            self._step_ackermann_dt(v_mag, delta, self.dt)
            guard += 1

    def _mecanum_turn_in_place(self, angle_rad: float, angular_speed: float, *, ccw: bool) -> None:
        assert self.mecanum is not None
        angular_speed = max(abs(angular_speed), 1e-6)
        remaining = abs(angle_rad)
        direction = 1.0 if ccw else -1.0
        k = self.mecanum.half_length_y + self.mecanum.half_width_x
        while remaining > 1e-6:
            dt_eff = min(self.dt, remaining / angular_speed)
            omega = direction * angular_speed
            a = omega * k
            self._step_mecanum_dt(-a, a, -a, a, dt_eff)
            remaining -= angular_speed * dt_eff

    def _turn_in_place(self, angle_rad: float, angular_speed: float, *, ccw: bool) -> None:
        angular_speed = max(abs(angular_speed), 1e-6)
        remaining = abs(angle_rad)
        direction = 1.0 if ccw else -1.0
        while remaining > 1e-6:
            dt_eff = min(self.dt, remaining / angular_speed)
            omega = direction * angular_speed
            if self.kind == KinematicsModel.DIFFERENTIAL:
                w = self.differential.track_width
                self._step_differential_wheels_dt(-omega * w / 2, omega * w / 2, dt_eff)
            else:
                self._step_unicycle_dt(0.0, omega, dt_eff)
            remaining -= angular_speed * dt_eff

    def rotate_to(self, theta_goal: float, angular_speed: float) -> None:
        """Rotate in place until heading equals *theta_goal* (two-wheel / differential / mecanum)."""
        if _is_ackermann(self.kind):
            raise NotImplementedError("Ackermann cannot rotate in place; use turn_left/right arcs")
        angular_speed = max(abs(angular_speed), 1e-6)
        if self.kind == KinematicsModel.MECANUM:
            while True:
                err = wrap_pi(theta_goal - self.pose.theta)
                if abs(err) < 1e-5:
                    break
                omega = max(-angular_speed, min(angular_speed, err / self.dt))
                k = self.mecanum.half_length_y + self.mecanum.half_width_x
                a = omega * k
                self._step_mecanum_dt(-a, a, -a, a, self.dt)
            return
        while True:
            err = wrap_pi(theta_goal - self.pose.theta)
            if abs(err) < 1e-5:
                break
            omega = max(-angular_speed, min(angular_speed, err / self.dt))
            if self.kind == KinematicsModel.DIFFERENTIAL:
                w = self.differential.track_width
                self._step_differential_wheels_dt(-omega * w / 2, omega * w / 2, self.dt)
            else:
                self._step_unicycle_dt(0.0, omega, self.dt)

    def drive_to_pose(
        self,
        target: Pose,
        *,
        linear_speed: float,
        angular_speed: float,
        position_tol: float = 0.06,
        angle_tol: float = 0.06,
        max_segments: int = 50,
    ) -> None:
        """Drive to *target* (two-wheel, differential, or mecanum omnidirectional)."""
        if _is_ackermann(self.kind):
            raise NotImplementedError("drive_to_pose is not implemented for Ackermann; use arcs.")
        linear_speed = abs(linear_speed)
        angular_speed = max(abs(angular_speed), 1e-6)
        if self.kind == KinematicsModel.MECANUM:
            self._drive_to_pose_mecanum(target, linear_speed, angular_speed, position_tol, angle_tol)
            return
        for _ in range(max_segments):
            dx = target.x - self.pose.x
            dy = target.y - self.pose.y
            dist = math.hypot(dx, dy)
            if dist <= position_tol:
                break
            h_goal = _heading_to_point(dx, dy)
            self.rotate_to(h_goal, angular_speed)
            self.forward(dist, linear_speed)
        else:
            raise RuntimeError("drive_to_pose: could not reach position")
        self.rotate_to(target.theta, angular_speed)
        if abs(wrap_pi(target.theta - self.pose.theta)) > angle_tol:
            raise RuntimeError("drive_to_pose: final orientation not reached")

    def _drive_to_pose_mecanum(
        self,
        target: Pose,
        linear_speed: float,
        angular_speed: float,
        position_tol: float,
        angle_tol: float,
    ) -> None:
        assert self.mecanum is not None
        max_seg = 80
        for _ in range(max_seg):
            dx = target.x - self.pose.x
            dy = target.y - self.pose.y
            dist = math.hypot(dx, dy)
            if dist <= position_tol:
                break
            h = _heading_to_point(dx, dy)
            err_theta = wrap_pi(h - self.pose.theta)
            if abs(err_theta) > 0.05:
                self.rotate_to(h, angular_speed)
            fwd = min(linear_speed * self.dt * 2, dist)
            self._mecanum_forward(fwd, linear_speed)
        else:
            raise RuntimeError("drive_to_pose (mecanum): position not reached")
        self.rotate_to(target.theta, angular_speed)
        if abs(wrap_pi(target.theta - self.pose.theta)) > angle_tol:
            raise RuntimeError("drive_to_pose (mecanum): orientation not reached")

    def forward_wheels(self, distance: float, wheel_speed: float) -> None:
        """Differential: both wheels same *wheel_speed* (m/s) for *distance* along path."""
        if self.kind != KinematicsModel.DIFFERENTIAL:
            raise TypeError("forward_wheels is only for differential drive")
        wheel_speed = abs(wheel_speed)
        remaining = abs(distance)
        while remaining > 1e-7:
            dt_eff = min(self.dt, remaining / wheel_speed)
            self._step_differential_wheels_dt(wheel_speed, wheel_speed, dt_eff)
            remaining -= wheel_speed * dt_eff

    def differential_drive_wheels(self, v_left: float, v_right: float, duration: float) -> None:
        """Differential: wheel speeds (m/s) for *duration* seconds."""
        if self.kind != KinematicsModel.DIFFERENTIAL:
            raise TypeError("differential_drive_wheels is only for differential drive")
        t = 0.0
        while t + 1e-12 < duration:
            dt_eff = min(self.dt, duration - t)
            self._step_differential_wheels_dt(v_left, v_right, dt_eff)
            t += dt_eff

    def mecanum_drive_wheels(
        self,
        v_fl: float,
        v_fr: float,
        v_rl: float,
        v_rr: float,
        duration: float,
    ) -> None:
        """Mecanum: wheel linear speeds (m/s) at contacts for *duration* seconds (FL, FR, RL, RR)."""
        if self.kind != KinematicsModel.MECANUM:
            raise TypeError("mecanum_drive_wheels is only for MECANUM")
        t = 0.0
        while t + 1e-12 < duration:
            dt_eff = min(self.dt, duration - t)
            self._step_mecanum_dt(v_fl, v_fr, v_rl, v_rr, dt_eff)
            t += dt_eff
