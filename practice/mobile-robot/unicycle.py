import AuroraMR as amr
import math

s = amr.MotionSession.create(
    amr.pose(0,0,0),
    amr.KinematicsModel.TWO_WHEEL,
    dt = 0.02,
    unicycle = amr.UnicycleParams(
        track_width=0.4, 
        max_linear_speed=1.5
    )
)

s.forward(1.0,speed = 0.8)
s.turn_left(math.pi/2, 1.0)
s.forward(0.5,speed = 0.8)

amr.plot_motion(s)
amr.play_motion(s,interval_ms = 25)