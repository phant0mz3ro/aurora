#!/usr/bin/env python3
"""Ackermann motion built in code, then played live at 0.5× speed with logging.

Shows the same parameters as :func:`amr.play_motion_by_kind` but with a custom path.

Run::

    PYTHONPATH=. python examples/live_ackermann_session.py

Optional: append logs to a file by setting ``LOG_PATH`` below.
"""

from __future__ import annotations

import math
import sys

import AuroraMR as amr #Aurora Library

# Set to a path string to see logs to a file, or None for stdout only
LOG_PATH: str | None = None #log file path


def main() -> None:
    params = amr.AckermannParams( #variables and constraints for our model
        wheelbase=0.55,
        track_width=0.36,
        max_steering_angle=0.5,
        max_speed=1.0,
    )
    session = amr.MotionSession.create( #creating the Ackermann model with our defined rules
        amr.pose(0.0, 0.0, 0.0), #starting position
        amr.KinematicsModel.ACKERMANN, #model
        dt=0.02, #discrete time steps
        ackermann=params,
    )
    session.forward(1.8, 0.5) #(distance,speed)
    session.turn_left(math.radians(45), 0.75) #(angle,ang_speed)
    session.forward(1.2, 0.45)

    log_stream = open(LOG_PATH, "w", encoding="utf-8") if LOG_PATH else None #gets file ready for write actions if file_path exists
    try:

        class Tee: #object class for writing multiple streams 
            def __init__(self, *streams: object) -> None:
                self.streams = streams

            def write(self, data: str) -> None:
                for s in self.streams:
                    s.write(data)
                    s.flush()

            def flush(self) -> None:
                for s in self.streams:
                    s.flush()

        out: object = Tee(sys.stdout, log_stream) if log_stream else sys.stdout #uses the file_path for output if it exists, else it uses the terminal directly

        opts = amr.PlaybackLogOptions( #calls the PlaybackLogOptions module that creates a well-formatted output
            enabled=True,
            every_n_frames=8,
            detailed_block=True,
            include_velocity=True,
            file=out,  # type: ignore[arg-type]
        )

        amr.play_motion( #play_motion runs the simulation
            session,
            interval_ms=30,
            playback_speed=0.5,
            title="Ackermann (custom path)",
            show=True,
            log_options=opts,
        )
    finally:
        if log_stream is not None:
            log_stream.close()


if __name__ == "__main__":
    main()
