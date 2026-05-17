#!/usr/bin/env python3
"""Real-time matplotlib animation with optional teaching logs in the terminal.

Examples::

    # All four models, normal speed, formatted logs for students
    python live_kinematics_demo.py all --log

    # 2× faster playback, detailed multi-line logs every 20 frames
    python live_kinematics_demo.py mecanum --speed 2 --log --log-every 20 --log-detailed

    # Save video (no window); logs still go to terminal unless --quiet-log
    python live_kinematics_demo.py all --save out.mp4 --log
"""

from __future__ import annotations

import argparse
import sys

import matplotlib
import matplotlib.animation
import matplotlib.pyplot as plt

import AuroraMR as amr #Aurora Library


def main() -> None:
    parser = argparse.ArgumentParser(description="Live AuroraMR kinematics demos with optional teaching logs")
    """
    argparse allows the user to specify arguments in the terminal to dictate the output of the program
    like the model, speed, time interval, log options,etc...
    """
    parser.add_argument( #what model do you want to use?
        "mode",
        nargs="?",
        default="all",
        choices=("all", "two_wheel", "unicycle", "differential", "ackermann", "mecanum"),
        help="Model to play, or 'all' for all four in sequence (default: all)",
    )
    parser.add_argument("--interval", type=int, default=28, help="Base milliseconds between frames (before speed)") #time interval?
    parser.add_argument( #speed, v of the body
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (>1 = faster, <1 = slower). Effective interval = interval/speed.",
    )
    #the help parameters helps communicate the function of the argument and its usage to the user
    parser.add_argument("--no-repeat", action="store_true", help="Do not loop the animation")
    parser.add_argument("--save", metavar="FILE", help="Save animation to file instead of showing")
    parser.add_argument("--log", action="store_true", help="Print teaching notes to the terminal while playing")
    parser.add_argument(
        "--log-every",
        type=int,
        default=12,
        metavar="N",
        help="Log every N simulation frames (frame 0 always logged if --log)",
    )
    parser.add_argument(
        "--log-detailed",
        action="store_true",
        help="Multi-line log blocks instead of one line per log step",
    )
    parser.add_argument(
        "--log-file",
        metavar="PATH",
        help="Append logs to this file as well as stdout (still prints to terminal unless --quiet-log)",
    )
    parser.add_argument(
        "--quiet-log",
        action="store_true",
        help="With --log-file, only write logs to the file (not stdout)",
    )
    args = parser.parse_args()

    if args.save:
        matplotlib.use("Agg")

    show_window = not args.save
    repeat = not args.no_repeat

    """
    log functionalities
    commented on something similar in the live_ackermann_session.py
    """
    log_file = None 
    log_stream = None
    if args.log_file:
        log_stream = open(args.log_file, "a", encoding="utf-8") #prepares the file for write operations if the path exists

    class _Tee: #the objects class for handling writing operations
        def __init__(self, *streams: object) -> None:
            self.streams = streams

        def write(self, data: str) -> None:
            for s in self.streams:
                s.write(data)
                s.flush()

        def flush(self) -> None:
            for s in self.streams:
                s.flush()

    if args.log:
        if args.quiet_log and log_stream is not None:
            tee: object = log_stream
        elif log_stream is not None:
            tee = _Tee(sys.stdout, log_stream)
        else:
            tee = sys.stdout #uses terminal output if file doesn't exist
        log_opts = amr.PlaybackLogOptions(  #module that returns a well formatted output of the object's state at each time step
            enabled=True,
            every_n_frames=max(1, args.log_every),
            detailed_block=args.log_detailed,
            file=tee,  # type: ignore[arg-type]
        )
    else:
        log_opts = None

    try: #dictates the demo program to use based on the mode specified in the terminal
        if args.mode == "all":
            anim = amr.play_all_kinematics_live(
                interval_ms=args.interval,
                playback_speed=args.speed,
                repeat=repeat,
                show=show_window,
                log_options=log_opts,
            )
        else:
            anim = amr.play_motion_by_kind(
                args.mode,
                interval_ms=args.interval,
                playback_speed=args.speed,
                show=show_window,
                log_options=log_opts,
            )

        if args.save and anim is not None:
            fps = max(1, int(round(1000 * args.speed / args.interval)))
            path = args.save
            if path.lower().endswith(".gif"):
                anim.save(path, writer=matplotlib.animation.PillowWriter(fps=fps))
            else:
                anim.save(path, writer=matplotlib.animation.FFMpegWriter(fps=fps))
            print("Saved", path)
            plt.close("all")
    finally:
        if log_stream is not None:
            log_stream.close()


if __name__ == "__main__":
    main()

"""
Baically, this program is an all-in-one kinematics demo program
the user can list any required specs as arguments in the terminal
and the program will play the corresponding demo with the specified options
smooth!
"""