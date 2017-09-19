import ikpy
import numpy as np
import trajgen as tg
import sympy as sp
# from ikpy import plot_utils
# import matplotlib.pyplot as plt

my_chain = ikpy.chain.Chain.from_urdf_file("6dof_description.urdf", base_elements=['base_link'], base_element_type='link')

# target_vector = [0.1, -0.2, 0.1]
# target_frame = np.eye(4)
theta0, theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta0 theta1 theta2 theta3 theta4 theta5')
theta = sp.Matrix([theta0, theta1, theta2, theta3, theta4, theta5])
symb_mat = my_chain.symbolic_transformation_matrix()
J = my_chain.jacobian_matrix(symb_mat)
# print(J)
p0_vector = [0.08, 0, 0.02]
p0_frame = np.eye(4)
p0_frame[:3, 3] = p0_vector
pf_vector = [0.14, 0.02, 0.1]
pf_frame = np.eye(4)
pf_frame[:3, 3] = pf_vector
tf = 10
dt = 0.1
eps = 1e-10
p_d, time = tg.Trajectory_Generation(dt, tf, 0.1 * tf, p0_vector, pf_vector, 'line')
q_0 = my_chain.inverse_kinematics(p0_frame)
if not q_0.all:
    raise ValueError("Initial Position out of Work Space!")
    exit(0)
q_d = np.zeros([6, len(time)])
print(q_0)

q_prev = q_0
q = np.zeros([7, len(time)])
q[:, 0] = q_0

for i in range(0, len(time)-1):
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
    q_prev = q_next
print(q)
# print("The angles of each joints are : ", my_chain.inverse_kinematics(target_frame))
# real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_frame))
# print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_frame[:3, 3]))


# ax = plot_utils.init_3d_figure()
# my_chain.plot(my_chain.inverse_kinematics(target_frame), ax, target=target_vector)
# plt.xlim(-0.1, 0.1)
# plt.ylim(-0.1, 0.1)
# plt.show()
