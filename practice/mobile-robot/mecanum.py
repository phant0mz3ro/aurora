import math
import AuroraMR as amr

s = amr.MotionSession.create(
    amr.pose(0,0,0),
    amr.KinematicsModel.MECANUM,
    dt  =  0.015,
    mecanum = amr.MecanumParams(
        half_length_y=0.26,
        half_width_x=0.18,
        max_wheel_speed=2.5
    )
)

s.forward(0.7,speed=0.6)
s.strafe_right(0.5,speed=0.55)
s.turn_left(math.pi/4, 1.0)

s.mecanum_drive_wheels(0.4,0.4,-0.0,0.0,duration=3)

amr.play_motion(s,interval_ms = 25,log=True,log_every_n_frames=10)