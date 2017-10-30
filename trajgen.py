
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import ikpy
import ikpy.geometry_utils as gu


def x(t):

    return np.array([1, t, t**2, t**3, t**4, t**5])


def x_l(t):

    return np.array([1, t])


def dx(t):

    return np.array([0, 1, 2 * t, 3 * t**2, 4 * t**3, 5 * t**4])


def dxdx(t):

    return np.array([0, 0, 2, 6 * t, 12 * t**2, 20 * t**3])


class Trajectory_Generator(object):

    def __init__(self, time_step, tf, Delta):
        self.time_step = time_step
        self.tf = tf
        self.Delta = Delta
        self.time = np.arange(0, self.tf + self.time_step, self.time_step)

    def polynomial_interpolation_with_steady_speed(self, p_zero, p_final):

        dp = float((p_final - p_zero)) / (self.tf - 2 * self.Delta)
        # print(dp)
        pf1 = p_final - dp * self.Delta
        p02 = p_zero + dp * self.Delta
        # print(pf1)
        # print(p02)
        A1 = np.array([x(0), x(2 * self.Delta), dx(0), dx(2 * self.Delta), dxdx(0), dxdx(2 * self.Delta)])
        B1 = np.array([p_zero, p02, 0, dp, 0, 0])
        sol1 = np.linalg.solve(A1, B1)
        # print(sol1)
        A2 = np.array([x_l(2 * self.Delta), [0, 1]])
        B2 = np.array([p02, dp])
        sol2 = np.linalg.solve(A2, B2)
        # print(sol2)
        A3 = np.array([x(self.tf - 2 * self.Delta), x(self.tf), dx(self.tf - 2 * self.Delta), dx(self.tf), dxdx(self.tf - 2 * self.Delta), dxdx(self.tf)])
        B3 = np.array([pf1, p_final, dp, 0, 0, 0])
        sol3 = np.linalg.solve(A3, B3)
        # print(sol3)
        p = np.zeros(len(self.time))
        d_p = np.zeros(len(self.time))
        dd_p = np.zeros(len(self.time))
        for j in range(0, len(self.time)):
            if (j * self.time_step <= 2 * self.Delta):
                # print(x(time[j]))
                # print(x(time[j]).shape)
                # print(sol1)
                p[j] = np.dot(x(self.time[j]), sol1)

                d_p[j] = np.dot(dx(self.time[j]), sol1)
                dd_p[j] = np.dot(dxdx(self.time[j]), sol1)
            else:
                if (j * self.time_step > 2 * self.Delta and j * self.time_step <= self.tf - 2 * self.Delta):
                    p[j] = np.dot(x_l(self.time[j]), sol2)
                    d_p[j] = np.dot([0, 1], sol2)
                    dd_p[j] = np.dot([0, 0], sol2)

                else:
                    p[j] = np.dot(x(self.time[j]), sol3)
                    d_p[j] = np.dot(dx(self.time[j]), sol3)
                    dd_p[j] = np.dot(dxdx(self.time[j]), sol3)
        return p, d_p, dd_p

    def angle_axis_generation(self, Ri, Rf):

        R_if = np.dot(Ri.T, Rf)
        theta_f, r = gu.angle_axis_from_rotation_matrix(R_if)
        # print(Rf, np.dot(Ri, gu.rotation_matrix_from_angle_axis(theta_f, *r)))
        # print(R_if, gu.rotation_matrix_from_angle_axis(theta_f, *r))
        theta_0 = 0
        # The answer to the below code is this: https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        if (theta_f >= theta_0 and (ikpy.geometry_utils.angle_difference(theta_f, theta_0) < 0)):
            theta_0 = theta_0 + 2 * np.pi
        elif (theta_f < theta_0 and (ikpy.geometry_utils.angle_difference(theta_f, theta_0) > 0)):
            theta_0 = theta_0 - 2 * np.pi
        print(theta_f, theta_0)
        lamda = float((theta_f - theta_0)) / self.tf

        flag = lamda == 0

        if flag:
            theta = np.zeros(len(self.time))
            d_theta = np.zeros(len(self.time))
            dd_theta = np.zeros(len(self.time))
            print("No change in orientation!")
            return theta, d_theta, r
        else:
            theta_zero = theta_0
            theta_final = theta_f

        if (self.Delta >= .5 * (self.tf - self.time[0])):
            print('The input you gave is not valid')
            exit(0)
        theta, d_theta, dd_theta = self.polynomial_interpolation_with_steady_speed(theta_zero, theta_final)

        # print(d_theta)
        # print(dd_theta)
        # plt.hold(False)
        # print(len(time))
        # fig2 = plt.figure()
        # ax2 = fig2.gca()
        # ax2.plot(time, theta, label="rotation", color='b')
        # ax2.plot(time, d_theta, label="1st derivative", color='y')
        # ax2.plot(time, dd_theta, label="2nd derivative", color='r')
        # ax2.title.set_text('Trajectory with derivatives')
        # ax2.legend()
        # ax2.set_xlabel('time(sec)')
        # plt.draw()
        # plt.show()

        # plt.hold(True)
        # plt.close(fig1)
        # plt.close(fig2)
        return np.array([theta]), np.array([d_theta]), r


# def euler_angles_generation(time_step, tf, Delta, theta0, thetaf):
#     time = np.arange(0, tf + time_step, time_step)
#     # Wrap angles to [-pi,pi)
#     theta0 = ikpy.geometry_utils.wrap_to_pi(theta0)
#     # theta0 = (theta0 + np.pi) % (2 * np.pi) - np.pi
#     thetaf = ikpy.geometry_utils.wrap_to_pi(thetaf)
#     # The answer to the below code is this: https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
#     if (thetaf[0] >= theta0[0] and (ikpy.geometry_utils.angle_difference(thetaf[0], theta0[0]) < 0)):
#         theta0[0] = theta0[0] + 2 * np.pi
#     elif (thetaf[0] < theta0[0] and (ikpy.geometry_utils.angle_difference(thetaf[0], theta0[0]) > 0)):
#         theta0[0] = theta0[0] - 2 * np.pi
#     if (thetaf[1] >= theta0[1] and (ikpy.geometry_utils.angle_difference(thetaf[1], theta0[1]) < 0)):
#         theta0[1] = theta0[1] + 2 * np.pi
#     elif (thetaf[1] < theta0[1] and (ikpy.geometry_utils.angle_difference(thetaf[1], theta0[1]) > 0)):
#         theta0[1] = theta0[1] - 2 * np.pi
#     if (thetaf[2] >= theta0[2] and (ikpy.geometry_utils.angle_difference(thetaf[2], theta0[2]) < 0)):
#         theta0[2] = theta0[2] + 2 * np.pi
#     elif (thetaf[2] < theta0[2] and (ikpy.geometry_utils.angle_difference(thetaf[2], theta0[2]) > 0)):
#         theta0[2] = theta0[2] - 2 * np.pi

#     lamda_a = float((thetaf[0] - theta0[0])) / tf
#     lamda_b = float((thetaf[1] - theta0[1])) / tf
#     lamda_c = float((thetaf[2] - theta0[2])) / tf

#     flag2 = lamda_a == 0

#     flag3 = lamda_b == 0
#     flag4 = lamda_c == 0

#     if flag2:
#         if flag3:
#             if flag4:
#                 theta_d = np.zeros((3, len(time)))
#                 dtheta_d = np.zeros((3, len(time)))
#                 ddtheta_d = np.zeros((3, len(time)))
#                 print("No change in orientation!")
#                 return theta_d, time
#             else:
#                 theta_zero = theta0[2]
#                 theta_final = thetaf[2]
#         else:
#             theta_zero = theta0[1]
#             theta_final = thetaf[1]
#     else:
#         theta_zero = theta0[0]
#         theta_final = thetaf[0]

#     if (Delta >= .5 * (tf - time[0])):
#         print('The input you gave is not valid')
#         exit(0)
#     dtheta = float((theta_final - theta_zero)) / (tf - 2 * Delta)
#     # print(dtheta)
#     thetaf1 = theta_final - dtheta * Delta
#     theta02 = theta_zero + dtheta * Delta
#     # print(thetaf1)
#     # print(theta02)
#     A1 = np.array([x(0), x(2 * Delta), dx(0), dx(2 * Delta), dxdx(0), dxdx(2 * Delta)])
#     B1 = np.array([theta_zero, theta02, 0, dtheta, 0, 0])
#     sol1 = np.linalg.solve(A1, B1)
#     # print(sol1)
#     A2 = np.array([x_l(2 * Delta), [0, 1]])
#     B2 = np.array([theta02, dtheta])
#     sol2 = np.linalg.solve(A2, B2)
#     # print(sol2)
#     A3 = np.array([x(tf - 2 * Delta), x(tf), dx(tf - 2 * Delta), dx(tf), dxdx(tf - 2 * Delta), dxdx(tf)])
#     B3 = np.array([thetaf1, theta_final, dtheta, 0, 0, 0])
#     sol3 = np.linalg.solve(A3, B3)
#     # print(sol3)
#     theta = np.zeros(len(time))
#     d_theta = np.zeros(len(time))
#     dd_theta = np.zeros(len(time))
#     theta_d = np.zeros((3, len(time)))
#     dtheta_d = np.zeros((3, len(time)))
#     ddtheta_d = np.zeros((3, len(time)))
#     for j in range(0, len(time)):
#         if (j * time_step <= 2 * Delta):
#             theta[j] = np.dot(x(time[j]), sol1)

#             d_theta[j] = np.dot(dx(time[j]), sol1)
#             dd_theta[j] = np.dot(dxdx(time[j]), sol1)
#         else:
#             if (j * time_step > 2 * Delta and j * time_step <= tf - 2 * Delta):
#                 theta[j] = np.dot(x_l(time[j]), sol2)
#                 d_theta[j] = np.dot([0, 1], sol2)
#                 dd_theta[j] = np.dot([0, 0], sol2)

#             else:
#                 theta[j] = np.dot(x(time[j]), sol3)
#                 d_theta[j] = np.dot(dx(time[j]), sol3)
#                 dd_theta[j] = np.dot(dxdx(time[j]), sol3)
#     # print(theta)
#     if flag2:
#         if flag3:
#             theta_d[2, :] = theta
#             dtheta_d[2, :] = d_theta
#             ddtheta_d[2, :] = dd_theta
#         else:
#             theta_d[1, :] = theta
#             dtheta_d[1, :] = d_theta
#             ddtheta_d[1, :] = dd_theta
#     else:
#         theta_d[0, :] = theta
#         dtheta_d[0, :] = d_theta
#         ddtheta_d[0, :] = dd_theta

#     # print(d_theta)
#     # print(dd_theta)
#     # fig1 = plt.figure()
#     # ax1 = fig1.gca(projection='3d')
#     # plt.ion()
#     for j in range(0, len(time)):

#         if flag2:
#             if flag3:
#                 theta_d[0, j] = theta0[0]
#                 theta_d[1, j] = theta0[1]
#             else:
#                 theta_d[0, j] = theta0[0]
#                 if flag4:
#                     theta_d[2, j] = theta0[2]
#                 else:
#                     theta_d[2, j] = lamda_c * (theta_d[1, j] - theta0[1]) / lamda_b + theta0[2]
#         else:
#             theta_d[1, j] = lamda_b * (theta_d[0, j] - theta0[0]) / lamda_a + theta0[1]
#             theta_d[2, j] = lamda_c * (theta_d[0, j] - theta0[0]) / lamda_a + theta0[2]
#     #     if(j % 2 == 0):
#     #         ax1.scatter(theta_d[0, j], theta_d[1, j], theta_d[2, j], '-.ob')
#     #         # axis([x_lim y_lim z_lim])
#     #         ax1.title.set_text('3D Trajectory Generated')
#     #         ax1.set_xlabel('a(rad)')
#     #         ax1.set_ylabel('b(rad)')
#     #         ax1.set_zlabel('c(rad)')
#     #         plt.draw()
#     #         plt.show()
#     #         plt.pause(0.05)
#     # # plt.hold(False)
#     # plt.ioff()
#     # print(len(time))
#     # fig2 = plt.figure()
#     # ax2 = fig2.gca()
#     # ax2.plot(time, theta, label="orientation", color='b')
#     # ax2.plot(time, d_theta, label="1st derivative", color='y')
#     # ax2.plot(time, dd_theta, label="2nd derivative", color='r')
#     # ax2.title.set_text('Trajectory with derivatives')
#     # ax2.legend()
#     # ax2.set_xlabel('time(sec)')
#     # plt.draw()
#     # plt.show()

#     # plt.hold(True)
#     # plt.close(fig1)
#     # plt.close(fig2)
#     return theta_d, time

    def line_generation(self, p0, pf):

        # helix code
        # r = 5
        # alpha = 5
        lamda_x = float((pf[0] - p0[0])) / self.tf
        lamda_y = float((pf[1] - p0[1])) / self.tf
        lamda_z = float((pf[2] - p0[2])) / self.tf

        flag2 = lamda_x == 0

        flag3 = lamda_y == 0
        flag4 = lamda_z == 0
        if flag2:
            if flag3:
                if flag4:
                    print('Wrong Starting and Final Points: ABORTING\n')
                    exit(0)
                else:
                    p_zero = p0[2]
                    p_final = pf[2]
            else:
                p_zero = p0[1]
                p_final = pf[1]
        else:
            p_zero = p0[0]
            p_final = pf[0]

        if (self.Delta >= .5 * (self.tf - self.time[0])):
            print('The input you gave is not valid')
            exit(0)
        p_d = np.zeros((3, len(self.time)))
        dp_d = np.zeros((3, len(self.time)))
        ddp_d = np.zeros((3, len(self.time)))
        p, d_p, dd_p = self.polynomial_interpolation_with_steady_speed(p_zero, p_final)
        # print(p)
        if flag2:
            if flag3:
                p_d[2, :] = p
                dp_d[2, :] = d_p
                ddp_d[2, :] = dd_p
            else:
                p_d[1, :] = p
                dp_d[1, :] = d_p
                ddp_d[1, :] = dd_p
        else:
            p_d[0, :] = p
            dp_d[0, :] = d_p
            ddp_d[0, :] = dd_p

        # print(d_p)
        # print(dd_p)
        # fig1 = plt.figure()
        # ax1 = fig1.gca(projection='3d')
        # plt.ion()
        for j in range(0, len(self.time)):
                # draw 3D line:
            if flag2:
                if flag3:
                    p_d[0, j] = p0[0]
                    p_d[1, j] = p0[1]
                else:
                    p_d[0, j] = p0[0]
                    if flag4:
                        p_d[2, j] = p0[2]
                    else:
                        p_d[2, j] = lamda_z * (p_d[1, j] - p0[1]) / lamda_y + p0[2]
                        dp_d[2, j] = lamda_z * (dp_d[1, j]) / lamda_y
                        ddp_d[2, j] = lamda_z * (ddp_d[1, j]) / lamda_y

            else:
                p_d[1, j] = lamda_y * (p_d[0, j] - p0[0]) / lamda_x + p0[1]
                p_d[2, j] = lamda_z * (p_d[0, j] - p0[0]) / lamda_x + p0[2]
                dp_d[1, j] = lamda_y * (dp_d[0, j]) / lamda_x
                dp_d[2, j] = lamda_z * (dp_d[0, j]) / lamda_x
                ddp_d[1, j] = lamda_y * (ddp_d[0, j]) / lamda_x
                ddp_d[2, j] = lamda_z * (ddp_d[0, j]) / lamda_x
        #     if(j % 2 == 0):
        #         ax1.scatter(p_d[0, j], p_d[1, j], p_d[2, j], '-.ob')
        #         # axis([x_lim y_lim z_lim])
        #         ax1.title.set_text('3D Trajectory Generated')
        #         ax1.set_xlabel('x(m)')
        #         ax1.set_ylabel('y(m)')
        #         ax1.set_zlabel('z(m)')
        #         plt.draw()
        #         plt.show()
        #         plt.pause(0.05)
        # # plt.hold(False)
        # plt.ioff()
        # print(len(time))
        # fig2 = plt.figure()
        # ax2 = fig2.gca()
        # ax2.plot(time, p, label="trajectory", color='b')
        # ax2.plot(time, d_p, label="1st derivative", color='y')
        # ax2.plot(time, dd_p, label="2nd derivative", color='r')
        # ax2.title.set_text('Trajectory with derivatives')
        # ax2.legend()
        # ax2.set_xlabel('time(sec)')
        # plt.draw()
        # plt.show()

        # plt.hold(True)
        # plt.close(fig1)
        # plt.close(fig2)
        return p_d, dp_d

    def circle_generation(self, p0, pf, pm):

        # helix code
        # r = 5
        # alpha = 5
        lamda_x = float((pf[0] - p0[0])) / self.tf
        lamda_y = float((pf[1] - p0[1])) / self.tf
        lamda_z = float((pf[2] - p0[2])) / self.tf
        epsilon = 1e-10
        flag2 = lamda_x == 0
        flag3 = lamda_y == 0
        flag4 = lamda_z == 0
        flag5 = np.linalg.norm(p0 - pm) < epsilon
        flag6 = np.linalg.norm(pm - pf) < epsilon
        n_prp_has_z = False
        n_prp_in_y = False
        n_prp_in_xy = False
        # p0 = np.array(p0)
        # pf = np.array(pf)
        # pm = np.array(pm)
        print(p0, pf, pm)
        if flag5 or flag6:
            print("You should give 3 different points\n")
            exit(0)
        if flag2:
            if flag3:
                if flag4:
                    print('Wrong Starting and Final Points: ABORTING\n')
                    exit(0)
        n_prp = np.cross(pm - p0, pf - p0)
        w = n_prp / np.linalg.norm(n_prp)
        d_plane = - np.inner(n_prp, pf)
        if(abs(w[2]) < epsilon):
            if(abs(w[0]) < epsilon):
                n_prp_in_y = True
            else:
                n_prp_in_xy = True
        else:
            n_prp_has_z = True

        print(n_prp, d_plane, w)
        pcenter = np.zeros(3)
        if n_prp_has_z:
            A = np.array([[2 * (p0[0] - pf[0] - n_prp[0] / n_prp[2] * (p0[2] - pf[2])), 2 * (p0[1] - pf[1] - n_prp[1] / n_prp[2] * (p0[2] - pf[2]))], [2 * (p0[0] - pm[0] - n_prp[0] / n_prp[2] * (p0[2] - pm[2])), 2 * (p0[1] - pm[1] - n_prp[1] / n_prp[2] * (p0[2] - pm[2]))]])
            B = np.array([[p0[0]**2 - pf[0]**2 + p0[1]**2 - pf[1]**2 + p0[2]**2 - pf[2]**2 + 2 * d_plane / n_prp[2] * (p0[2] - pf[2])], [p0[0]**2 - pm[0]**2 + p0[1]**2 - pm[1]**2 + p0[2]**2 - pm[2]**2 + 2 * d_plane / n_prp[2] * (p0[2] - pm[2])]])
            pcenter[:2] = np.linalg.solve(A, B).ravel()
            pcenter[2] = -(d_plane + n_prp[0] * pcenter[0] + n_prp[1] * pcenter[1]) / n_prp[2]
        else:
            if n_prp_in_xy:
                A = np.array([[2 * (p0[1] - pf[1] - n_prp[1] / n_prp[0] * (p0[0] - pf[0])), 2 * (p0[2] - pf[2])], [2 * (p0[1] - pm[1] - n_prp[1] / n_prp[0] * (p0[0] - pm[0])), 2 * (p0[2] - pm[2])]])
                B = np.array([[p0[0]**2 - pf[0]**2 + p0[1]**2 - pf[1]**2 + p0[2]**2 - pf[2]**2 + 2 * d_plane / n_prp[0] * (p0[0] - pf[0])], [p0[0]**2 - pm[0]**2 + p0[1]**2 - pm[1]**2 + p0[2]**2 - pm[2]**2 + 2 * d_plane / n_prp[0] * (p0[0] - pm[0])]])
                pcenter[1:] = np.linalg.solve(A, B).ravel()
                pcenter[0] = -(d_plane + n_prp[1] * pcenter[1]) / n_prp[0]
            elif n_prp_in_y:
                pcenter[1] = -(d_plane) / n_prp[1]
                A = np.array([[2 * (p0[0] - pf[0]), 2 * (p0[2] - pf[2])], [2 * (p0[0] - pm[0]), 2 * (p0[2] - pm[2])]])
                B = np.array([[p0[0]**2 - pf[0]**2 + p0[2]**2 - pf[2]**2], [p0[0]**2 - pm[0]**2 + p0[2]**2 - pm[2]**2]])
                pcenter[0:3:2] = np.linalg.solve(A, B).ravel()
        print(A, B)
        print(pcenter)
        Rcenter = np.linalg.norm(p0 - pcenter)
        print(pcenter, Rcenter)
        u = (p0 - pcenter) / np.linalg.norm(p0 - pcenter)
        v = np.cross(w, u)
        print(u, v, w)
        R_plane = np.vstack((u, v, w)).T
        print(R_plane)
        # exit(0)
        # print(np.linalg.norm(pf - pcenter), np.linalg.norm(pm - pcenter))
        # exit(0)
        # theta_offset = np.arccos((p0 - pcenter) / Rcenter)
        # theta_offset_1 = np.arcsin((p0[1] - pcenter[1]) / Rcenter)
        # print(theta_offset, theta_offset_1)
        theta_zero = 0
        theta_zero_final = np.arccos(np.inner(p0 - pcenter, pf - pcenter) / (np.linalg.norm(p0 - pcenter) * np.linalg.norm(pf - pcenter)))
        theta_zero_middle = np.arccos(np.inner(p0 - pcenter, pm - pcenter) / (np.linalg.norm(p0 - pcenter) * np.linalg.norm(pm - pcenter)))
        theta_middle_final = np.arccos(np.inner(pm - pcenter, pf - pcenter) / (np.linalg.norm(pm - pcenter) * np.linalg.norm(pf - pcenter)))
        print(theta_zero_final, theta_zero_middle, theta_middle_final)

        theta_final = theta_zero_final if theta_zero_middle + theta_middle_final <= np.pi else 2 * np.pi - theta_zero_final
        print(theta_final)
        # pf_from_plane = pcenter[:2] + np.dot(R_plane[:2, :2], np.array([Rcenter * np.cos(theta_final), Rcenter * np.sin(theta_final)]))
        # zf_from_plane = -(d_plane + n_prp[0] * pf_from_plane[0] + n_prp[1] * pf_from_plane[1]) / n_prp[2]
        # print(pf_from_plane, zf_from_plane)
        # exit(0)
        if (self.Delta >= .5 * (self.tf - self.time[0])):
            print('The input you gave is not valid')
            exit(0)
        theta, d_theta, dd_theta = self.polynomial_interpolation_with_steady_speed(theta_zero, theta_final)
        p_d = np.zeros((3, len(self.time)))
        dp_d = np.zeros((3, len(self.time)))
        ddp_d = np.zeros((3, len(self.time)))
        fig1 = plt.figure()
        ax1 = fig1.gca(projection='3d')
        plt.ion()
        for j in range(0, len(self.time)):

            # p_d[0, j] = pcenter[0] + Rcenter * np.cos(theta[j])
            # p_d[1, j] = pcenter[1] + Rcenter * np.sin(theta[j])
            # p_d[2, j] = -(d_plane + n_prp[0] * p_d[0, j] + n_prp[1] * p_d[1, j]) / n_prp[2]
            if n_prp_has_z:
                x_plane = Rcenter * np.cos(theta[j])
                y_plane = Rcenter * np.sin(theta[j])
                p_plane = np.array([x_plane, y_plane])
                p_d[:2, j] = pcenter[:2] + np.dot(R_plane[:2, :2], p_plane)
                p_d[2, j] = -(d_plane + n_prp[0] * p_d[0, j] + n_prp[1] * p_d[1, j]) / n_prp[2]
                dx_plane = -Rcenter * np.sin(theta[j]) * d_theta[j]
                dy_plane = Rcenter * np.cos(theta[j]) * d_theta[j]
                dp_plane = np.array([dx_plane, dy_plane])
                dp_d[:2, j] = np.dot(R_plane[:2, :2], dp_plane)
                dp_d[2, j] = -(n_prp[0] * dp_d[0, j] + n_prp[1] * dp_d[1, j]) / n_prp[2]
                ddx_plane = -Rcenter * np.cos(theta[j]) * dd_theta[j]
                ddy_plane = -Rcenter * np.sin(theta[j]) * dd_theta[j]
                ddp_plane = np.array([ddx_plane, ddy_plane])
                ddp_d[:2, j] = np.dot(R_plane[:2, :2], ddp_plane)
                ddp_d[2, j] = -(n_prp[0] * ddp_d[0, j] + n_prp[1] * ddp_d[1, j]) / n_prp[2]
            else:
                if n_prp_in_xy:
                    z_plane = Rcenter * np.cos(theta[j])
                    y_plane = Rcenter * np.sin(theta[j])
                    p_plane = np.array([z_plane, y_plane])
                    p_d[1:, j] = pcenter[1:] + np.dot(R_plane[1:, :2], p_plane)
                    p_d[0, j] = -(d_plane + n_prp[1] * p_d[1, j]) / n_prp[0]
                    dz_plane = -Rcenter * np.sin(theta[j]) * d_theta[j]
                    dy_plane = Rcenter * np.cos(theta[j]) * d_theta[j]
                    dp_plane = np.array([dz_plane, dy_plane])
                    dp_d[1:, j] = np.dot(R_plane[1:, :2], dp_plane)
                    dp_d[0, j] = -(n_prp[1] * dp_d[1, j]) / n_prp[0]
                    ddz_plane = -Rcenter * np.cos(theta[j]) * dd_theta[j]
                    ddy_plane = -Rcenter * np.sin(theta[j]) * dd_theta[j]
                    ddp_plane = np.array([ddz_plane, ddy_plane])
                    ddp_d[1:, j] = np.dot(R_plane[1:, :2], ddp_plane)
                    ddp_d[0, j] = -(n_prp[1] * ddp_d[1, j]) / n_prp[0]
                elif n_prp_in_y:
                    z_plane = Rcenter * np.cos(theta[j])
                    x_plane = Rcenter * np.sin(theta[j])
                    p_plane = np.array([z_plane, x_plane])
                    p_d[0:3:2, j] = pcenter[0:3:2] + np.dot(R_plane[0:3:2, :2], p_plane)
                    p_d[1, j] = -(d_plane) / n_prp[1]
                    dz_plane = -Rcenter * np.sin(theta[j]) * d_theta[j]
                    dx_plane = Rcenter * np.cos(theta[j]) * d_theta[j]
                    dp_plane = np.array([dz_plane, dx_plane])
                    dp_d[0:3:2, j] = np.dot(R_plane[0:3:2, :2], dp_plane)
                    dp_d[1, j] = 0
                    ddz_plane = -Rcenter * np.cos(theta[j]) * dd_theta[j]
                    ddx_plane = -Rcenter * np.sin(theta[j]) * dd_theta[j]
                    ddp_plane = np.array([ddz_plane, ddx_plane])
                    ddp_d[0:3:2, j] = np.dot(R_plane[0:3:2, :2], ddp_plane)
                    ddp_d[1, j] = 0

            if(j % 2 == 0):
                ax1.scatter(p_d[0, j], p_d[1, j], p_d[2, j], '-.ob')
                # axis([x_lim y_lim z_lim])
                ax1.title.set_text('3D Trajectory Generated')
                ax1.set_xlabel('x(m)')
                ax1.set_ylabel('y(m)')
                ax1.set_zlabel('z(m)')
                ax1.set_xlim([-0.5, 0.5])
                ax1.set_ylim([-0.5, 0.5])
                ax1.set_zlim([-0.5, 0.5])
                plt.draw()
                plt.show()
                plt.pause(0.05)
        print(p_d[0, :])
        print(p_d[1, :])
        print(p_d[2, :])
        # plt.hold(False)
        plt.ioff()
        fig2 = plt.figure()
        ax2 = fig2.gca()
        ax2.plot(self.time, p_d[0, :], label="trajectory", color='b')
        ax2.plot(self.time, dp_d[0, :], label="1st derivative", color='y')
        ax2.plot(self.time, ddp_d[0, :], label="2nd derivative", color='r')
        ax2.title.set_text('Trajectory with derivatives')
        ax2.legend()
        ax2.set_xlabel('time(sec)')
        plt.draw()
        plt.show()

        plt.hold(True)
        # plt.close(fig1)
        # plt.close(fig2)
        # print(p_d, dp_d)
        # exit(0)
        return p_d, dp_d
