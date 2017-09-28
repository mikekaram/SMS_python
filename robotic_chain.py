import ikpy
import numpy as np
import trajgen as tg
import sympy as sp
# from ikpy import plot_utils
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D as ax3
import time
import sms_library_oo as sms
import time as t
import ikpy.geometry_utils as gu
global absolute_position_homing, limits_ticks, resolution_bits, motorIds

# limits_ticks = np.array([[1062, 1330], [192, 5921], [909, 4498], [], [17054, 7374], []])
# print(len(limits_ticks[1]))

# absolute_positions_homing = np.array([11145, 11692, 6942, 6876, 30025, 22205])
# def get_angle_difference(a1, b1, c1, a2, b2, c2):
#     a1 = ikpy.geometry_utils.wrap_to_pi(a1)
#     b1 = ikpy.geometry_utils.wrap_to_pi(b1)
#     c1 = ikpy.geometry_utils.wrap_to_pi(c1)
#     a2 = ikpy.geometry_utils.wrap_to_pi(a2)
#     b2 = ikpy.geometry_utils.wrap_to_pi(b2)
#     c2 = ikpy.geometry_utils.wrap_to_pi(c2)
#     d1 = ikpy.geometry_utils.angle_difference(a2, a1)
#     d2 = ikpy.geometry_utils.angle_difference(b2, b1)
#     d3 = ikpy.geometry_utils.angle_difference(c2, c1)
#     return np.array([d1, d2, d3])


class Robot_Chain(object):

    def __init__(self, urdf_file, motors):
        self.robot_chain = ikpy.chain.Chain.from_urdf_file(urdf_file, base_elements=['base_link'], base_element_type='link')
        theta0, theta1, theta2, theta3, theta4, theta5 = sp.symbols('theta0 theta1 theta2 theta3 theta4 theta5')
        self.theta = sp.Matrix([theta0, theta1, theta2, theta3, theta4, theta5])
        self.transition_matrix = self.robot_chain.symbolic_transformation_matrix(6)
        self.J = self.robot_chain.jacobian_matrix(self.transition_matrix)
        self.motors = motors

    def move_xyz_abc(self, pf, a, b, c):
        p0_frame = self.robot_chain.forward_kinematics([0] * 7)
        # p0_vector = p0_frame[:3][3]
        pf_frame = np.eye(4)
        pf_frame[:3, :3] = gu.rpy_matrix(a, b, c)
        pf_vector = pf
        pf_frame[:3, 3] = pf_vector
        print(p0_frame)
        print(pf_frame)
        tf = 10
        dt = 0.1
        Delta = 0.1 * tf
        # eps = 1e-10
        t_d, dt_d, r, simulation_time = tg.trajectory_generation(dt, tf, Delta, p0_frame, pf_frame)
        q, q_d = self.motion_control(p0_frame, pf_frame, t_d, dt_d, r, simulation_time, dt)
        # for i, motor in enumerate(self.motors):
        #     print(int(gu.angle_to_ticks(q[i, :], motor.resolution_bits)))
        # self.animate_move(q, q_d, p0_frame[:3, 3], pf_frame[:3, 3], simulation_time)

    def motion_control(self, p0_frame, pf_frame, t_d, dt_d, r, simulation_time, dt):

        damping = 0.005
        K_o = 2
        K_p = 2 * np.eye(3)
        q_0 = self.robot_chain.inverse_kinematics(p0_frame)
        # p_0 = self.robot_chain.forward_kinematics(q_0)
        if not q_0.all:
            raise ValueError("Initial Position out of Work Space!")
            exit(0)
        q_d = np.zeros([6, len(simulation_time)])
        print(q_0)

        q_prev = q_0
        q = np.zeros([7, len(simulation_time)])
        q[:, 0] = q_0
        damping_coef = damping**2 * np.eye(6)
        t = time.time()
        for i in range(0, len(simulation_time) - 1):
            self.move_motors(q_prev)
            p_star_frame = self.robot_chain.forward_kinematics(q_prev)
            omega_i = dt_d[3, i] * r
            omega_e = np.dot(p0_frame[:3, :3], omega_i)
            # print(omega_e.shape)
            ita_e, e_e = gu.quaternion_from_rotation_matrix(p_star_frame[:3, :3])
            ita_d, e_d = gu.quaternion_from_rotation_matrix(np.dot(p0_frame[:3, :3], gu.rotation_matrix_from_angle_axis(t_d[3, i], *r)))
            e_e = np.reshape(e_e, (1, -1))
            e_d = np.reshape(e_d, (1, -1))
            # print(e_e, e_d)
            e_o = ita_e * e_d - ita_d * e_e - np.cross(e_d, e_e)
            e_o = np.reshape(e_o, (-1, 1))
            omega_e = np.reshape(omega_e, (-1, 1))
            print(ita_e * ita_d + np.inner(e_e, e_d), e_o)
            thetadot = omega_e + K_o * e_o
            e_p = t_d[:3, i] - p_star_frame[:3, 3]
            pdot = dt_d[:3, i] + np.dot(K_p, e_p)
            pdot = np.reshape(pdot, (-1, 1))
            thetadot = np.reshape(thetadot, (-1, 1))
            tdot = np.concatenate((pdot, thetadot))
            print("T dot is:", tdot)
            Ji = self.J.subs([(self.theta[0], q_prev[1]), (self.theta[1], q_prev[2]), (self.theta[2], q_prev[3]), (self.theta[3], q_prev[4]), (self.theta[4], q_prev[5]), (self.theta[5], q_prev[6])])
            Ji = np.array(Ji).astype(np.float64)
            # solution of inverse kinematics using dps from http://math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
            J_ = np.dot(Ji, np.transpose(Ji))
            f = np.linalg.solve(J_ + damping_coef, tdot)
            qdot = np.dot(np.transpose(Ji), f)
            q_next = q_prev + np.insert(qdot, 0, 0) * dt
            q[:, i + 1] = q_next
            q_d[:, i] = qdot.ravel()
            q_prev = q_next
            print("Elapsed time is:", time.time() - t)
        return q, q_d

    def move_motors(self, q):
        for i, motor in enumerate(self.motors):
            if i == 2:
                motor.setProfiledAbsolutePositionSetpoint(-int(gu.angle_to_ticks(q[i], motor.resolution_bits)))
            else:
                motor.setProfiledAbsolutePositionSetpoint(int(gu.angle_to_ticks(q[i], motor.resolution_bits)))
            t.sleep(0.003)
        sms.broadcastDoMove()

    def homing(self):
        ticks_to_go = np.zeros((6, 1))
        sms.broadcastStart()
        t.sleep(0.02)
        absolute_positions_entry_positions = np.zeros((6, 1))
        for i, motor in enumerate(self.motors):
            mId = motor.motorId
            print(i, mId)
            absolute_positions_entry_positions[i] = motor.getAbsolutePosition()[1]
            print(absolute_positions_entry_positions)
            t.sleep(0.01)
            absolute_angle_homing = gu.ticks_to_angle(motor.homing_position_ticks, motor.resolution_bits)
            absolute_angle_entry = gu.ticks_to_angle(absolute_positions_entry_positions[i], motor.resolution_bits)
            absolute_angle_limits = gu.ticks_to_angle(motor.limits, motor.resolution_bits)
            angles_to_go_limits = gu.angle_difference_with_limits(absolute_angle_homing, absolute_angle_entry, absolute_angle_limits)
            angles_to_go = gu.angle_difference(absolute_angle_homing, absolute_angle_entry)
            print(angles_to_go_limits)
            print(angles_to_go)
            ticks_to_go[i] = gu.angle_to_ticks(angles_to_go_limits, motor.resolution_bits)
            print(ticks_to_go)
            motor.setProfiledAbsolutePositionSetpoint(int(ticks_to_go[i]))
            t.sleep(0.04)
        sms.broadcastDoMove()
        t.sleep(5)
        sms.broadcastStart()

    def animate_move(self, q, q_d, p0_vector, pf_vector, simulation_time):
        print(q, q_d, p0_vector, pf_vector, simulation_time)
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
            for index, link in enumerate(self.robot_chain.links):
                position["pos%d" % (index + 1)][:, j] = np.array(self.robot_chain.symbolic_transformation_matrix(index + 1).subs([(self.theta[0], q[1, j]), (self.theta[1], q[2, j]), (self.theta[2], q[3, j]), (self.theta[3], q[4, j]), (self.theta[4], q[5, j]), (self.theta[5], q[6, j])])[:3, 3]).astype(np.float64).ravel()
        print(position)

        min_x = np.amin(np.concatenate([position["pos0"][0, :], position["pos1"][0, :], position["pos2"][0, :], position["pos3"][0, :], position["pos4"][0, :], position["pos5"][0, :], position["pos6"][0, :], position["pos7"][0, :]]))
        max_x = np.amax(np.concatenate([[position["pos0"][0, :], position["pos1"][0, :], position["pos2"][0, :], position["pos3"][0, :], position["pos4"][0, :], position["pos5"][0, :], position["pos6"][0, :], position["pos7"][0, :]]]))
        min_y = np.amin(np.concatenate([[position["pos0"][1, :], position["pos1"][1, :], position["pos2"][1, :], position["pos3"][1, :], position["pos4"][1, :], position["pos5"][1, :], position["pos6"][1, :], position["pos7"][1, :]]]))
        max_y = np.amax(np.concatenate([[position["pos0"][1, :], position["pos1"][1, :], position["pos2"][1, :], position["pos3"][1, :], position["pos4"][1, :], position["pos5"][1, :], position["pos6"][1, :], position["pos7"][1, :]]]))
        # min_z = np.amin(np.concatenate([[position["pos0"][2, :], position["pos1"][2, :], position["pos2"][2, :], position["pos3"][2, :], position["pos4"][2, :], position["pos5"][2, :], position["pos6"][2, :], position["pos7"][2, :]]]))
        # max_z = np.amax(np.concatenate([[position["pos0"][2, :], position["pos1"][2, :], position["pos2"][2, :], position["pos3"][2, :], position["pos4"][2, :], position["pos5"][2, :], position["pos6"][2, :], position["pos7"][2, :]]]))

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
