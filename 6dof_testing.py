import ikpy
import numpy as np
import trajgen as tg
import sympy as sp
from ikpy import plot_utils
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D as ax3

my_chain = ikpy.chain.Chain.from_urdf_file("6dof_description.urdf", base_elements=['base_link'], base_element_type='link')
print(my_chain.forward_kinematics([0] * 7))
# target_vector = [0.1, -0.2, 0.1]
# target_frame = np.eye(4)
theta0, theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta0 theta1 theta2 theta3 theta4 theta5')
theta = sp.Matrix([theta0, theta1, theta2, theta3, theta4, theta5])
symb_mat = my_chain.symbolic_transformation_matrix(6)
# print(symb_mat)
# print(symb_mat.subs([(theta0, 0), (theta1, 0), (theta2, 0), (theta3, 0), (theta4, 0), (theta5, 0)]))
J = my_chain.jacobian_matrix(symb_mat)
# print(J.subs([(theta0, 0), (theta1, 0), (theta2, 0), (theta3, 0), (theta4, 0), (theta5, 0)]))
# print(J)
p0_vector = [0.08, 0, 0.02]
p0_frame = np.eye(4)
p0_frame[:3, 3] = p0_vector
pf_vector = [0, 0, 0.3]
pf_frame = np.eye(4)
pf_frame[:3, 3] = pf_vector
tf = 10
dt = 0.1
eps = 1e-10
p_d, time = tg.Trajectory_Generation(dt, tf, 0.1 * tf, p0_vector, pf_vector, 'line')
q_0 = my_chain.inverse_kinematics(p0_frame)
p_0 = my_chain.forward_kinematics(q_0)
print(p_0)
if not q_0.all:
    raise ValueError("Initial Position out of Work Space!")
    exit(0)
q_d = np.zeros([6, len(time)])
print(q_0)

q_prev = q_0
q = np.zeros([7, len(time)])
q[:, 0] = q_0

for i in range(0, len(time) - 1):
    p_star = my_chain.forward_kinematics(q_prev)
    p_star = p_star[:3, 3]
    pdot = (p_d[:, i] - p_star) / dt
    Ji = J.subs([(theta0, q_prev[1]), (theta1, q_prev[2]), (theta2, q_prev[3]), (theta3, q_prev[4]), (theta4, q_prev[5]), (theta5, q_prev[6])])
    # print(Ji)
    # print(Ji.shape)
    # print(type(Ji))
    # if abs(np.linalg.det(np.array(Ji).astype(np.float64))) < eps:
    #     raise ValueError("Singularity in Jacobian!!! Simulation Terminated")
    #     exit(0)
    Ji = Ji[:3, :]
    qdot = np.dot(np.linalg.pinv(np.array(Ji).astype(np.float64)), pdot)
    # print(qdot)
    q_next = q_prev + np.insert(qdot, 0, 0) * dt
    q_next = (q_next + np.pi) % (2 * np.pi) - np.pi
    q[:, i + 1] = q_next
    q_d[:, i] = qdot
    # print("pdot is:", pdot)
    q_prev = q_next
# position0 = np.zeros([3, len(q[0, :])])
# position1 = np.zeros([3, len(q[0, :])])
# position2 = np.zeros([3, len(q[0, :])])
# position3 = np.zeros([3, len(q[0, :])])
# position4 = np.zeros([3, len(q[0, :])])
# position5 = np.zeros([3, len(q[0, :])])
# position6 = np.zeros([3, len(q[0, :])])
# position7 = np.zeros([3, len(q[0, :])])
# position = {"pos0": position0, "pos1": position1, "pos2": position2, "pos3": position3, "pos4": position4, "pos5": position5, "pos6": position6, "pos7": position7}

# print(len(my_chain.links))
# for j in range(0, len(q[0, :])):
#     for index, link in enumerate(my_chain.links):
#         position["pos%d" % (index + 1)][:, j] = np.array(my_chain.symbolic_transformation_matrix(index + 1).subs([(theta0, q[1, j]), (theta1, q[2, j]), (theta2, q[3, j]), (theta3, q[4, j]), (theta4, q[5, j]), (theta5, q[6, j])])[:3, 3]).astype(np.float64).ravel()
# print(position)

# min_x = min(position["pos0"][0, :], position["pos1"][0, :], position["pos2"][0, :], position["pos3"][0, :], position["pos4"][0, :], position["pos5"][0, :], position["pos6"][0, :])
# max_x = max([position["pos0"][0, :], position["pos1"][0, :], position["pos2"][0, :], position["pos3"][0, :], position["pos4"][0, :], position["pos5"][0, :], position["pos6"][0, :]])
# min_y = min([position["pos0"][1, :], position["pos1"][1, :], position["pos2"][1, :], position["pos3"][1, :], position["pos4"][1, :], position["pos5"][1, :], position["pos6"][1, :]])
# max_y = max([position["pos0"][1, :], position["pos1"][1, :], position["pos2"][1, :], position["pos3"][1, :], position["pos4"][1, :], position["pos5"][1, :], position["pos6"][1, :]])
# min_z = min([position["pos0"][2, :], position["pos1"][2, :], position["pos2"][2, :], position["pos3"][2, :], position["pos4"][2, :], position["pos5"][2, :], position["pos6"][2, :]])
# max_z = max([position["pos0"][2, :], position["pos1"][2, :], position["pos2"][2, :], position["pos3"][2, :], position["pos4"][2, :], position["pos5"][2, :], position["pos6"][2, :]])

for i in range(0, len(q[0, :])):
    ax = plot_utils.init_3d_figure()
    my_chain.plot(list([q[0, i], q[1, i], q[2, i], q[3, i], q[4, i], q[5, i], q[6, i]]), ax)
    plt.xlim(-0.3, 0.3)
    plt.ylim(-0.3, 0.3)
    plt.draw()
    plt.show()
# print(position)
# for i in range(0, len(q[0, :])):
#     pos = np.vstack(position["pos0"][:, i], position["pos1"][:, i], position["pos2"][:, i], position["pos3"][:, i], position["pos4"][:, i], position["pos5"][:, i], position["pos6"][:, i])
#     fig = plt.figure()
#     ax = fig.gca(projection='3d')

#     ax.plot([p0_vector[0], pf_vector[0]], [p0_vector[1], pf_vector[1]], [p0_vector[2], pf_vector[2]], color='green', linewidth=2, marker='o')
#     ax.plot([pos(1, 1) pos(2, 1)], [pos(1, 2) pos(2, 2)], [pos(1, 3) pos(2, 3)], 'Color', 'b', 'linewidth', 2, 'Marker', 'o')
#     line(simulator, [pos(2, 1) pos(3, 1)], [pos(2, 2) pos(3, 2)], [pos(2, 3) pos(3, 3)], 'Color', 'g', 'linewidth', 2, 'Marker', 'o')
#     line(simulator, [pos(3, 1) pos(4, 1)], [pos(3, 2) pos(4, 2)], [pos(3, 3) pos(4, 3)], 'Color', 'r', 'linewidth', 2, 'Marker', 'o')
#     line(simulator, [pos(4, 1) pos(5, 1)], [pos(4, 2) pos(5, 2)], [pos(4, 3) pos(5, 3)], 'Color', [1 0.5 0], 'linewidth', 2, 'Marker', 'o')
#     line(simulator, [pos(5, 1) pos(6, 1)], [pos(5, 2) pos(6, 2)], [pos(5, 3) pos(6, 3)], 'Color', 'm', 'linewidth', 2, 'Marker', 'o')
#     line(simulator, [pos(6, 1) pos(7, 1)], [pos(6, 2) pos(7, 2)], [pos(6, 3) pos(7, 3)], 'Color', 'c', 'linewidth', 2, 'Marker', 'o')
#     axis([min_x max_x min_y max_y min_z max_z])
#     title(simulator, 'Arm Kinematic Simulation')
#     xlabel('x (cm)')
#     ylabel('y (cm)')
#     zlabel('z (cm)')
#     grid on
#     drawnow limitrate
#     hold off
# print(q)
# print("The angles of each joints are : ", my_chain.inverse_kinematics(target_frame))
# real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_frame))
# print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_frame[:3, 3]))


# ax = plot_utils.init_3d_figure()
# my_chain.plot(my_chain.inverse_kinematics(target_frame), ax, target=target_vector)
# plt.xlim(-0.1, 0.1)
# plt.ylim(-0.1, 0.1)
# plt.show()
