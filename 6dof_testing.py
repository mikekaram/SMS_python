import ikpy
import numpy as np
import trajgen as tg
import sympy as sp
# from ikpy import plot_utils
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D as ax3
import time

my_chain = ikpy.chain.Chain.from_urdf_file("6dof_description.urdf", base_elements=['base_link'], base_element_type='link')
print(my_chain.forward_kinematics([0] * 7))
# target_vector = [0.1, -0.2, 0.1]
# target_frame = np.eye(4)
theta0, theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta0 theta1 theta2 theta3 theta4 theta5')
theta = sp.Matrix([theta0, theta1, theta2, theta3, theta4, theta5])
symb_mat = my_chain.symbolic_transformation_matrix(6)
# print(symb_mat)
# print("Initial Position from Symbolic:", symb_mat.subs([(theta0, 0), (theta1, 0), (theta2, 0), (theta3, 0), (theta4, 0), (theta5, 0)]))

# print("Initial Position by IK:", p0_vector)
J = my_chain.jacobian_matrix(symb_mat)
# print(J.subs([(theta0, 0), (theta1, 0), (theta2, 0), (theta3, 0), (theta4, 0), (theta5, 0)]))
# print(J)
p0_vector = [0.06, 0, 0]
# p0_vector = my_chain.forward_kinematics([0] * 7)[:3, 3]
# print("p0_vector:", p0_vector)
p0_frame = np.eye(4)
p0_frame[:3, 3] = p0_vector
pf_vector = [-0.05, -0.14, 0.17]
pf_frame = np.eye(4)
pf_frame[:3, 3] = pf_vector
tf = 10
dt = 0.1
eps = 1e-10
p_d, simulation_time = tg.Trajectory_Generation(dt, tf, 0.1 * tf, p0_vector, pf_vector, 'line')
q_0 = my_chain.inverse_kinematics(p0_frame)
p_0 = my_chain.forward_kinematics(q_0)
print(p_0)
if not q_0.all:
    raise ValueError("Initial Position out of Work Space!")
    exit(0)
q_d = np.zeros([6, len(simulation_time)])
print(q_0)

q_prev = q_0
# q_prev = (q_prev + np.pi) % (2 * np.pi) - np.pi
q = np.zeros([7, len(simulation_time)])
q[:, 0] = q_0
damping = 0.001
damping_coef = damping**2 * np.eye(3)
t = time.time()
for i in range(0, len(simulation_time) - 1):
    # p_star = symb_mat.subs([(theta0, q_prev[1]), (theta1, q_prev[2]), (theta2, q_prev[3]), (theta3, q_prev[4]), (theta4, q_prev[5]), (theta5, q_prev[6])])
    # p_star = np.array([p_star[3], p_star[7], p_star[11]]).astype(np.float64)
    p_star = my_chain.forward_kinematics(q_prev)[:3, 3]
    pdot = (p_d[:, i] - p_star) / dt
    # print("P dot is:", pdot)
    Ji = J.subs([(theta0, q_prev[1]), (theta1, q_prev[2]), (theta2, q_prev[3]), (theta3, q_prev[4]), (theta4, q_prev[5]), (theta5, q_prev[6])])
    # print(Ji)
    # print(Ji.shape)
    # print(type(Ji))
    # if abs(np.linalg.det(np.array(Ji).astype(np.float64))) < eps:
    #     raise ValueError("Singularity in Jacobian!!! Simulation Terminated")
    #     exit(0)
    Ji = np.array(Ji[:3, :]).astype(np.float64)
    # solution of inverse kinematics using dps from http://math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
    J_ = np.dot(Ji, np.transpose(Ji))
    # print("Ji is:", Ji)
    # print("J_ is:", J_)
    f = np.linalg.solve(J_ + damping_coef, pdot)
    qdot = np.dot(np.transpose(Ji), f)
    # qdot = np.dot(np.linalg.pinv(np.array(Ji).astype(np.float64)), pdot)
    # print(qdot)
    q_next = q_prev + np.insert(qdot, 0, 0) * dt
    # q_next = (q_next + np.pi) % (2 * np.pi) - np.pi
    q[:, i + 1] = q_next
    q_d[:, i] = qdot
    # print("pdot is:", pdot)
    q_prev = q_next
    print("Elapsed time is:", time.time() - t)
position0 = np.zeros([3, len(q[0, :])])
position1 = np.zeros([3, len(q[0, :])])
position2 = np.zeros([3, len(q[0, :])])
position3 = np.zeros([3, len(q[0, :])])
position4 = np.zeros([3, len(q[0, :])])
position5 = np.zeros([3, len(q[0, :])])
position6 = np.zeros([3, len(q[0, :])])
position7 = np.zeros([3, len(q[0, :])])
position = {"pos0": position0, "pos1": position1, "pos2": position2, "pos3": position3, "pos4": position4, "pos5": position5, "pos6": position6, "pos7": position7}

# print(len(my_chain.links))
for j in range(0, len(q[0, :])):
    for index, link in enumerate(my_chain.links):
        position["pos%d" % (index + 1)][:, j] = np.array(my_chain.symbolic_transformation_matrix(index + 1).subs([(theta0, q[1, j]), (theta1, q[2, j]), (theta2, q[3, j]), (theta3, q[4, j]), (theta4, q[5, j]), (theta5, q[6, j])])[:3, 3]).astype(np.float64).ravel()
print(position)
print(q)
min_x = np.amin(np.concatenate([position["pos0"][0, :], position["pos1"][0, :], position["pos2"][0, :], position["pos3"][0, :], position["pos4"][0, :], position["pos5"][0, :], position["pos6"][0, :], position["pos7"][0, :]]))
max_x = np.amax(np.concatenate([[position["pos0"][0, :], position["pos1"][0, :], position["pos2"][0, :], position["pos3"][0, :], position["pos4"][0, :], position["pos5"][0, :], position["pos6"][0, :], position["pos7"][0, :]]]))
min_y = np.amin(np.concatenate([[position["pos0"][1, :], position["pos1"][1, :], position["pos2"][1, :], position["pos3"][1, :], position["pos4"][1, :], position["pos5"][1, :], position["pos6"][1, :], position["pos7"][1, :]]]))
max_y = np.amax(np.concatenate([[position["pos0"][1, :], position["pos1"][1, :], position["pos2"][1, :], position["pos3"][1, :], position["pos4"][1, :], position["pos5"][1, :], position["pos6"][1, :], position["pos7"][1, :]]]))
min_z = np.amin(np.concatenate([[position["pos0"][2, :], position["pos1"][2, :], position["pos2"][2, :], position["pos3"][2, :], position["pos4"][2, :], position["pos5"][2, :], position["pos6"][2, :], position["pos7"][2, :]]]))
max_z = np.amax(np.concatenate([[position["pos0"][2, :], position["pos1"][2, :], position["pos2"][2, :], position["pos3"][2, :], position["pos4"][2, :], position["pos5"][2, :], position["pos6"][2, :], position["pos7"][2, :]]]))

fig = plt.figure()
plt.ion()
ax = fig.gca(projection='3d')
# ax.hold(False)
for i in range(0, len(q[0, :])):
    pos = np.hstack((np.reshape(position["pos0"][:, i], (-1, 1)), np.reshape(position["pos1"][:, i], (-1, 1)), np.reshape(position["pos2"][:, i], (-1, 1)), np.reshape(position["pos3"][:, i], (-1, 1)), np.reshape(position["pos4"][:, i], (-1, 1)), np.reshape(position["pos5"][:, i], (-1, 1)), np.reshape(position["pos6"][:, i], (-1, 1)), np.reshape(position["pos7"][:, i], (-1, 1))))
    # print(pos.shape)
    ax.plot([p0_vector[0], pf_vector[0]], [p0_vector[1], pf_vector[1]], [p0_vector[2], pf_vector[2]], color='green', linewidth=2, marker='o')
    ax.plot([pos[0, 0], pos[0, 1]], [pos[1, 0], pos[1, 1]], [pos[2, 0], pos[2, 1]], color='blue', linewidth=2, marker='o')
    ax.plot([pos[0, 1], pos[0, 2]], [pos[1, 1], pos[1, 2]], [pos[2, 1], pos[2, 2]], color='green', linewidth=2, marker='o')
    ax.plot([pos[0, 2], pos[0, 3]], [pos[1, 2], pos[1, 3]], [pos[2, 2], pos[2, 3]], color='black', linewidth=2, marker='o')
    ax.plot([pos[0, 3], pos[0, 4]], [pos[1, 3], pos[1, 4]], [pos[2, 3], pos[2, 4]], color='yellow', linewidth=2, marker='o')
    ax.plot([pos[0, 4], pos[0, 5]], [pos[1, 4], pos[1, 5]], [pos[2, 4], pos[2, 5]], color='blue', linewidth=2, marker='o')
    ax.plot([pos[0, 5], pos[0, 6]], [pos[1, 5], pos[1, 6]], [pos[2, 5], pos[2, 6]], color='green', linewidth=2, marker='o')
    ax.plot([pos[0, 6], pos[0, 7]], [pos[1, 6], pos[1, 7]], [pos[2, 6], pos[2, 7]], color='red', linewidth=2, marker='o')
    plt.ylim(min_y, max_y)
    plt.xlim(min_x, max_x)
    # plt.zlim(min_z, max_z)
    plt.title('Arm Kinematic Simulation')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    # plt.zlabel('z (cm)')
    plt.draw()
    plt.show()
    plt.pause(0.05)
    if (i == len(q[0, :]) - 1):
        plt.ioff()
        plt.hold(True)
        # while(True):
        #     plt.pause(0.05)
    else:
        plt.cla()


plt.ioff()
fig2 = plt.figure()
ax2 = fig2.gca()
ax2.plot(simulation_time, q[1, :], label="q0", color='b')
ax2.plot(simulation_time, q_d[1, :], label="1st derivative of q0", color='g')
ax2.title.set_text('Node Angle with Angular Velocity')
ax2.legend()
ax2.set_xlabel('time(sec)')
plt.draw()
plt.show()
fig2 = plt.figure()
ax2 = fig2.gca()
ax2.plot(simulation_time, q[2, :], label="q1", color='b')
ax2.plot(simulation_time, q_d[2, :], label="1st derivative of q1", color='g')
ax2.title.set_text('Node Angle with Angular Velocity')
ax2.legend()
ax2.set_xlabel('time(sec)')
plt.draw()
plt.show()
fig2 = plt.figure()
ax2 = fig2.gca()
ax2.plot(simulation_time, q[3, :], label="q2", color='b')
ax2.plot(simulation_time, q_d[3, :], label="1st derivative of q2", color='g')
ax2.title.set_text('Node Angle with Angular Velocity')
ax2.legend()
ax2.set_xlabel('time(sec)')
plt.draw()
plt.show()
fig2 = plt.figure()
ax2 = fig2.gca()
ax2.plot(simulation_time, q[4, :], label="q3", color='b')
ax2.plot(simulation_time, q_d[1, :], label="1st derivative of q3", color='g')
ax2.title.set_text('Node Angle with Angular Velocity')
ax2.legend()
ax2.set_xlabel('time(sec)')
plt.draw()
plt.show()
fig2 = plt.figure()
ax2 = fig2.gca()
ax2.plot(simulation_time, q[4, :], label="q4", color='b')
ax2.plot(simulation_time, q_d[4, :], label="1st derivative of q4", color='g')
ax2.title.set_text('Node Angle with Angular Velocity')
ax2.legend()
ax2.set_xlabel('time(sec)')
plt.draw()
plt.show()
fig2 = plt.figure()
ax2 = fig2.gca()
ax2.plot(simulation_time, q[5, :], label="q5", color='b')
ax2.plot(simulation_time, q_d[5, :], label="1st derivative of q5", color='g')
ax2.title.set_text('Node Angle with Angular Velocity')
ax2.legend()
ax2.set_xlabel('time(sec)')
plt.draw()
plt.show()
