import sms_library_oo as sms
import numpy as np
import time as t
# import ikpy
import robotic_chain as r_chain
# sms.init(1)

# sms.broadcastStop()
limits_ticks = np.array([[1062, 1330], [192, 5921], [909, 4498], [], [17054, 7374], []])
# print(len(limits_ticks[1]))

absolute_positions_homing = np.array([11145, 11692, 6942, 6876, 30025, 22205])
motorIds = [4, 5, 6, 7, 8, 9]
resolution_bits = [14, 14, 14, 14, 15, 15]
motors = list()
# sms.broadcastStart()
for i in range(6):
    motor = sms.sms_motor(motorIds[i], resolution_bits[i], limits_ticks[i], absolute_positions_homing[i])
    motors.append(motor)
    # motor.resetErrors()
    # if i == 3:
    #     motor.setProfiledVelocitySetpoint(256)
    #     sms.broadcastDoMove()
    #     t.sleep(20)
t.sleep(2)
# # sms.broadcastStop()
robot = r_chain.Robot_Chain("6dof_description.urdf", motors)
# robot.homing()
of = [-np.pi / 2, 0, 0]
# pf = [0.06, -0.19, 0.06]
pf = [-0.14, -0.03, 0.2]
pm = [0.02, -0.02, 0.30]
# robot.move_xyz_abc(pf, *of)
robot.move_circ(pm, pf, *of)
