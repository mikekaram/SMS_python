import sms_library_oo as sms
import numpy as np
import time as t
# import ikpy
import robotic_chain as r_chain
# sms.init(1)

# sms.broadcastStop()
limits_ticks = np.array([[9606, 9754], [192, 5921], [909, 4498], [], [17054, 7374], []])
# print(len(limits_ticks[1]))

absolute_positions_homing = np.array([2734, 11700, 7000, 6622, 29925, 22493])
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

# pf = [0.06, -0.19, 0.06]
# pf = [-0.15, -0.0, 0.2]
# pm = [0, -0.16, 0.2]
# p0 = [0.18, 0.0, 0.25]
# good points for circle #1
# of = [-np.pi / 4, np.pi / 2, 0]
# pf = [-0.175, 0.0, 0.175]
# pm = [0.0, -0.175, 0.175]
# good points for circle #2
# of = [0, -np.pi / 2, 0]
# pf = [0.165, 0.0, 0.245]
# pm = [0.165, -0.05, 0.215]
# p0 = [0.165, -0.05, 0.175]
of = [-np.pi / 4, np.pi / 2, 0]
pf = [-0.165, 0.0, 0.165]
pm = [0.0, -0.165, 0.165]
p0 = [0.16, -0.02, 0.16]
# robot.move_xyz_abc(pf, *of)
robot.move_circ(p0, pm, pf, *of)
# sms.broadcastStop()
robot = r_chain.Robot_Chain("6dof_description.urdf", motors)
# robot.homing()
# of = [0, 0, 0]
# # of = [0, 0, 0]
# pf = [0, 0, 0.38]
# robot.move_xyz_abc(pf, *of)
