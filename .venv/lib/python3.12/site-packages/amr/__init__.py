"""AuroraMR — autonomous mobile robotics helpers.

Use ``import amr`` or ``import AuroraMR as amr`` (same API).
"""

from .pose import Pose, pose

__all__ = [
    "Pose",
    "pose",
    "simulate",
    "MotionSession",
    "plot_motion",
    "play_motion",
    "play_all_kinematics_live",
    "play_motion_by_kind",
    "PlaybackLogOptions",
    "effective_interval_ms",
    "KinematicsKind",
    "KinematicsModel",
    "UnicycleParams",
    "TwoWheelParams",
    "DifferentialParams",
    "AckermannParams",
    "BicycleParams",
    "MecanumParams",
]


def __getattr__(name: str):
    if name == "simulate":
        from .simulate import simulate

        return simulate
    if name == "MotionSession":
        from .motion import MotionSession

        return MotionSession
    if name == "plot_motion":
        from .plot_motion import plot_motion

        return plot_motion
    if name == "play_motion":
        from .live_motion import play_motion

        return play_motion
    if name == "play_all_kinematics_live":
        from .live_motion import play_all_kinematics_live

        return play_all_kinematics_live
    if name == "play_motion_by_kind":
        from .live_motion import play_motion_by_kind

        return play_motion_by_kind
    if name == "PlaybackLogOptions":
        from .playback_log import PlaybackLogOptions

        return PlaybackLogOptions
    if name == "effective_interval_ms":
        from .playback_log import effective_interval_ms

        return effective_interval_ms
    if name == "KinematicsKind":
        from .kinematics.params import KinematicsKind

        return KinematicsKind
    if name == "KinematicsModel":
        from .kinematics.params import KinematicsModel

        return KinematicsModel
    if name == "UnicycleParams":
        from .kinematics.params import UnicycleParams

        return UnicycleParams
    if name == "TwoWheelParams":
        from .kinematics.params import TwoWheelParams

        return TwoWheelParams
    if name == "DifferentialParams":
        from .kinematics.params import DifferentialParams

        return DifferentialParams
    if name == "AckermannParams":
        from .kinematics.params import AckermannParams

        return AckermannParams
    if name == "BicycleParams":
        from .kinematics.params import BicycleParams

        return BicycleParams
    if name == "MecanumParams":
        from .kinematics.params import MecanumParams

        return MecanumParams
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted({*globals(), *__all__})
