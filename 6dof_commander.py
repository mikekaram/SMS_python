import sms_library_oo as sms
import numpy as np
import time as t
# import ikpy
import robotic_chain as r_chain
import ikpy.geometry_utils as gu
sms.init(0)

# sms.broadcastStop()
limits_ticks = np.array([[9606, 9754], [192, 5921], [909, 4498], [], [17054, 7374], []])
# print(len(limits_ticks[1]))

absolute_positions_homing = np.array([2734, 11700, 7000, 6622, 29925, 22493])
# limits_ticks = np.array([[1062, 1330], [192, 5921], [909, 4498], [], [17054, 7374], []])
# print(len(limits_ticks[1]))

# absolute_positions_homing = np.array([11145, 11692, 6942, 6876, 30025, 22205])
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
robot.homing()

# pf = [0.06, -0.19, 0.06]
# pf = [-0.15, -0.0, 0.2]
# pm = [0, -0.16, 0.2]
# p0 = [0.18, 0.0, 0.25]

####################################

# good points for circle #1
# of = [-np.pi / 4, np.pi / 2, 0]
# pf = [-0.175, 0.0, 0.175]
# pm = [0.0, -0.175, 0.175]
# good points for circle #2
# of = [0, -np.pi / 2, 0]
# pf = [0.165, 0.0, 0.245]
# pm = [0.165, -0.05, 0.215]
# p0 = [0.165, -0.05, 0.175]

##################################


######################################
#  testing bench
# good points for line #1
p0 = robot.robot_chain.forward_kinematics([0] * 7)[:3, 3]
of = gu.angles_from_rotation_matrix(robot.robot_chain.forward_kinematics([0] * 7)[:3, :3])
pf = [0.1947, -0.001, 0.11]
robot.move_xyz_abc(p0, pf, *of)
t.sleep(1)
# good points for circle #1
p0 = [0.1947, -0.001, 0.11]
# of = [np.pi, -np.pi / 2, 0]  # rotation around local x-axis by pi rad - not so good response though...
of = [0, 0, -np.pi / 2]
pf = [-0.1303, 0.021, 0.165]
pm = [0.0347, -0.144, 0.165]
robot.move_circ(p0, pm, pf, *of)
t.sleep(1)
# good points for line #2
# p0 = [-0.1303, 0.021, 0.165]
# of = [np.pi / 10, 0, -np.pi / 2]  # same with circle #1
# # pf = [-0.1153, 0.1876, 0.17]
# pf = [-0.11, 0.15, 0.1]
######################################


# of = [-np.pi / 2, np.pi / 2, 0]
# of = gu.angles_from_rotation_matrix(robot.robot_chain.forward_kinematics([0] * 7)[:3, :3])
# print(of)
# robot.move_xyz_abc(p0, pf, *of)
# robot.move_circ(p0, pm, pf, *of)
# sms.broadcastStop()
# robot = r_chain.Robot_Chain("6dof_description.urdf", motors)
# robot.homing()
# of = [0, 0, 0]
# # of = [0, 0, 0]
# pf = [0, 0, 0.38]
# robot.move_xyz_abc(pf, *of)
