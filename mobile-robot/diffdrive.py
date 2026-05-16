import AuroraMR as amr
import math

s = amr.MotionSession.create(
    amr.pose(0,0,0),
    amr.KinematicsModel.DIFFERENTIAL,
    dt = 0.015,
    differential = amr.DifferentialParams(
        track_width=0.4, 
        wheel_radius=0.1,
        max_wheel_speed = 2.0
        )
)

s.forward(1.0,speed = 0.8) # move 1 m forward
s.turn_right(math.pi/2, 1.0) # turn 90 degrees to the right
s.differential_drive_wheels(0.3,0.8,duration=1.2)
  

amr.play_motion(s,interval_ms = 25,log=True,log_every_n_frames=10)