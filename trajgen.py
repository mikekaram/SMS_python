
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import ikpy


def x(t):

    return np.array([1, t, t**2, t**3, t**4, t**5])


def x_l(t):

    return np.array([1, t])


def dx(t):

    return np.array([0, 1, 2 * t, 3 * t**2, 4 * t**3, 5 * t**4])


def dxdx(t):

    return np.array([0, 0, 2, 6 * t, 12 * t**2, 20 * t**3])


def angle_axis_generation(time_step, tf, Delta, Ri, Rf):
    time = np.arange(0, tf + time_step, time_step)
    R_if = np.transpose(Ri) * Rf
    theta_f, r = ikpy.geometry_utils.angle_axis_from_rotation_matrix(R_if)
    theta_0 = 0
    # r = np.array([[R_if[2,1]-R_if[1,2]], [R_if[0,2]-R_if[2,0]], [R_if[1,0]-R_if[0,1]]]) / (2*np.sin(theta_f))

    # The answer to the below code is this: https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    if (theta_f >= theta_0 and (ikpy.geometry_utils.angle_difference(theta_f, theta_0) < 0)):
        theta_0 = theta_0 + 2 * np.pi
    elif (theta_f < theta_0 and (ikpy.geometry_utils.angle_difference(theta_f, theta_0) > 0)):
        theta_0 = theta_0 - 2 * np.pi

    lamda = float((theta_f - theta_0)) / tf

    flag = lamda == 0

    if flag:
        theta = np.zeros(len(time))
        d_theta = np.zeros(len(time))
        dd_theta = np.zeros(len(time))
        print("No change in orientation!")
        return theta, d_theta, r, time
    else:
        theta_zero = theta_0
        theta_final = theta_f

    if (Delta >= .5 * (tf - time[0])):
        print('The input you gave is not valid')
        exit(0)
    dtheta = float((theta_final - theta_zero)) / (tf - 2 * Delta)
    # print(dtheta)
    thetaf1 = theta_final - dtheta * Delta
    theta02 = theta_zero + dtheta * Delta
    # print(thetaf1)
    # print(theta02)
    A1 = np.array([x(0), x(2 * Delta), dx(0), dx(2 * Delta), dxdx(0), dxdx(2 * Delta)])
    B1 = np.array([theta_zero, theta02, 0, dtheta, 0, 0])
    sol1 = np.linalg.solve(A1, B1)
    # print(sol1)
    A2 = np.array([x_l(2 * Delta), [0, 1]])
    B2 = np.array([theta02, dtheta])
    sol2 = np.linalg.solve(A2, B2)
    # print(sol2)
    A3 = np.array([x(tf - 2 * Delta), x(tf), dx(tf - 2 * Delta), dx(tf), dxdx(tf - 2 * Delta), dxdx(tf)])
    B3 = np.array([thetaf1, theta_final, dtheta, 0, 0, 0])
    sol3 = np.linalg.solve(A3, B3)
    # print(sol3)
    theta = np.zeros(len(time))
    d_theta = np.zeros(len(time))
    dd_theta = np.zeros(len(time))
    for j in range(0, len(time)):
        if (j * time_step <= 2 * Delta):
            theta[j] = np.dot(x(time[j]), sol1)
            d_theta[j] = np.dot(dx(time[j]), sol1)
            dd_theta[j] = np.dot(dxdx(time[j]), sol1)
        else:
            if (j * time_step > 2 * Delta and j * time_step <= tf - 2 * Delta):
                theta[j] = np.dot(x_l(time[j]), sol2)
                d_theta[j] = np.dot([0, 1], sol2)
                dd_theta[j] = np.dot([0, 0], sol2)

            else:
                theta[j] = np.dot(x(time[j]), sol3)
                d_theta[j] = np.dot(dx(time[j]), sol3)
                dd_theta[j] = np.dot(dxdx(time[j]), sol3)
    # print(theta)

    # print(d_theta)
    # print(dd_theta)
    # fig1 = plt.figure()
    # ax1 = fig1.gca(projection='3d')
    # plt.ion()
    # for j in range(0, len(time)):
    #     if(j % 2 == 0):
    #         ax1.scatter(theta_d[j],time[j],'-.ob')
    #         # axis([x_lim y_lim z_lim])
    #         ax1.title.set_text('Theta Trajectory Generated')
    #         ax1.set_xlabel('theta(rad)')
    #         ax1.set_ylabel('time(rad)')
    #         plt.draw()
    #         plt.show()
    #         plt.pause(0.05)
    # # plt.hold(False)
    # plt.ioff()
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
    return theta, d_theta, r, time


def euler_angles_generation(time_step, tf, Delta, theta0, thetaf):
    time = np.arange(0, tf + time_step, time_step)
    # Wrap angles to [-pi,pi)
    theta0 = ikpy.geometry_utils.wrap_to_pi(theta0)
    # theta0 = (theta0 + np.pi) % (2 * np.pi) - np.pi
    thetaf = ikpy.geometry_utils.wrap_to_pi(thetaf)
    # The answer to the below code is this: https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    if (thetaf[0] >= theta0[0] and (ikpy.geometry_utils.angle_difference(thetaf[0], theta0[0]) < 0)):
        theta0[0] = theta0[0] + 2 * np.pi
    elif (thetaf[0] < theta0[0] and (ikpy.geometry_utils.angle_difference(thetaf[0], theta0[0]) > 0)):
        theta0[0] = theta0[0] - 2 * np.pi
    if (thetaf[1] >= theta0[1] and (ikpy.geometry_utils.angle_difference(thetaf[1], theta0[1]) < 0)):
        theta0[1] = theta0[1] + 2 * np.pi
    elif (thetaf[1] < theta0[1] and (ikpy.geometry_utils.angle_difference(thetaf[1], theta0[1]) > 0)):
        theta0[1] = theta0[1] - 2 * np.pi
    if (thetaf[2] >= theta0[2] and (ikpy.geometry_utils.angle_difference(thetaf[2], theta0[2]) < 0)):
        theta0[2] = theta0[2] + 2 * np.pi
    elif (thetaf[2] < theta0[2] and (ikpy.geometry_utils.angle_difference(thetaf[2], theta0[2]) > 0)):
        theta0[2] = theta0[2] - 2 * np.pi

    lamda_a = float((thetaf[0] - theta0[0])) / tf
    lamda_b = float((thetaf[1] - theta0[1])) / tf
    lamda_c = float((thetaf[2] - theta0[2])) / tf

    flag2 = lamda_a == 0

    flag3 = lamda_b == 0
    flag4 = lamda_c == 0

    if flag2:
        if flag3:
            if flag4:
                theta_d = np.zeros((3, len(time)))
                dtheta_d = np.zeros((3, len(time)))
                ddtheta_d = np.zeros((3, len(time)))
                print("No change in orientation!")
                return theta_d, time
            else:
                theta_zero = theta0[2]
                theta_final = thetaf[2]
        else:
            theta_zero = theta0[1]
            theta_final = thetaf[1]
    else:
        theta_zero = theta0[0]
        theta_final = thetaf[0]

    if (Delta >= .5 * (tf - time[0])):
        print('The input you gave is not valid')
        exit(0)
    dtheta = float((theta_final - theta_zero)) / (tf - 2 * Delta)
    # print(dtheta)
    thetaf1 = theta_final - dtheta * Delta
    theta02 = theta_zero + dtheta * Delta
    # print(thetaf1)
    # print(theta02)
    A1 = np.array([x(0), x(2 * Delta), dx(0), dx(2 * Delta), dxdx(0), dxdx(2 * Delta)])
    B1 = np.array([theta_zero, theta02, 0, dtheta, 0, 0])
    sol1 = np.linalg.solve(A1, B1)
    # print(sol1)
    A2 = np.array([x_l(2 * Delta), [0, 1]])
    B2 = np.array([theta02, dtheta])
    sol2 = np.linalg.solve(A2, B2)
    # print(sol2)
    A3 = np.array([x(tf - 2 * Delta), x(tf), dx(tf - 2 * Delta), dx(tf), dxdx(tf - 2 * Delta), dxdx(tf)])
    B3 = np.array([thetaf1, theta_final, dtheta, 0, 0, 0])
    sol3 = np.linalg.solve(A3, B3)
    # print(sol3)
    theta = np.zeros(len(time))
    d_theta = np.zeros(len(time))
    dd_theta = np.zeros(len(time))
    theta_d = np.zeros((3, len(time)))
    dtheta_d = np.zeros((3, len(time)))
    ddtheta_d = np.zeros((3, len(time)))
    for j in range(0, len(time)):
        if (j * time_step <= 2 * Delta):
            theta[j] = np.dot(x(time[j]), sol1)

            d_theta[j] = np.dot(dx(time[j]), sol1)
            dd_theta[j] = np.dot(dxdx(time[j]), sol1)
        else:
            if (j * time_step > 2 * Delta and j * time_step <= tf - 2 * Delta):
                theta[j] = np.dot(x_l(time[j]), sol2)
                d_theta[j] = np.dot([0, 1], sol2)
                dd_theta[j] = np.dot([0, 0], sol2)

            else:
                theta[j] = np.dot(x(time[j]), sol3)
                d_theta[j] = np.dot(dx(time[j]), sol3)
                dd_theta[j] = np.dot(dxdx(time[j]), sol3)
    # print(theta)
    if flag2:
        if flag3:
            theta_d[2, :] = theta
            dtheta_d[2, :] = d_theta
            ddtheta_d[2, :] = dd_theta
        else:
            theta_d[1, :] = theta
            dtheta_d[1, :] = d_theta
            ddtheta_d[1, :] = dd_theta
    else:
        theta_d[0, :] = theta
        dtheta_d[0, :] = d_theta
        ddtheta_d[0, :] = dd_theta

    # print(d_theta)
    # print(dd_theta)
    # fig1 = plt.figure()
    # ax1 = fig1.gca(projection='3d')
    # plt.ion()
    for j in range(0, len(time)):

        if flag2:
            if flag3:
                theta_d[0, j] = theta0[0]
                theta_d[1, j] = theta0[1]
            else:
                theta_d[0, j] = theta0[0]
                if flag4:
                    theta_d[2, j] = theta0[2]
                else:
                    theta_d[2, j] = lamda_c * (theta_d[1, j] - theta0[1]) / lamda_b + theta0[2]
        else:
            theta_d[1, j] = lamda_b * (theta_d[0, j] - theta0[0]) / lamda_a + theta0[1]
            theta_d[2, j] = lamda_c * (theta_d[0, j] - theta0[0]) / lamda_a + theta0[2]
    #     if(j % 2 == 0):
    #         ax1.scatter(theta_d[0, j], theta_d[1, j], theta_d[2, j], '-.ob')
    #         # axis([x_lim y_lim z_lim])
    #         ax1.title.set_text('3D Trajectory Generated')
    #         ax1.set_xlabel('a(rad)')
    #         ax1.set_ylabel('b(rad)')
    #         ax1.set_zlabel('c(rad)')
    #         plt.draw()
    #         plt.show()
    #         plt.pause(0.05)
    # # plt.hold(False)
    # plt.ioff()
    # print(len(time))
    # fig2 = plt.figure()
    # ax2 = fig2.gca()
    # ax2.plot(time, theta, label="orientation", color='b')
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
    return theta_d, time


def line_generation(time_step, tf, Delta, p0, pf):

    time = np.arange(0, tf + time_step, time_step)
    # helix code
    # r = 5
    # alpha = 5
    lamda_x = float((pf[0] - p0[0])) / tf
    lamda_y = float((pf[1] - p0[1])) / tf
    lamda_z = float((pf[2] - p0[2])) / tf

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

    if (Delta >= .5 * (tf - time[0])):
        print('The input you gave is not valid')
        exit(0)
    dp = float((p_final - p_zero)) / (tf - 2 * Delta)
    # print(dp)
    pf1 = p_final - dp * Delta
    p02 = p_zero + dp * Delta
    # print(pf1)
    # print(p02)
    A1 = np.array([x(0), x(2 * Delta), dx(0), dx(2 * Delta), dxdx(0), dxdx(2 * Delta)])
    B1 = np.array([p_zero, p02, 0, dp, 0, 0])
    sol1 = np.linalg.solve(A1, B1)
    # print(sol1)
    A2 = np.array([x_l(2 * Delta), [0, 1]])
    B2 = np.array([p02, dp])
    sol2 = np.linalg.solve(A2, B2)
    # print(sol2)
    A3 = np.array([x(tf - 2 * Delta), x(tf), dx(tf - 2 * Delta), dx(tf), dxdx(tf - 2 * Delta), dxdx(tf)])
    B3 = np.array([pf1, p_final, dp, 0, 0, 0])
    sol3 = np.linalg.solve(A3, B3)
    # print(sol3)
    p = np.zeros(len(time))
    d_p = np.zeros(len(time))
    dd_p = np.zeros(len(time))
    p_d = np.zeros((3, len(time)))
    dp_d = np.zeros((3, len(time)))
    ddp_d = np.zeros((3, len(time)))
    for j in range(0, len(time)):
        if (j * time_step <= 2 * Delta):
            # print(x(time[j]))
            # print(x(time[j]).shape)
            # print(sol1)
            p[j] = np.dot(x(time[j]), sol1)

            d_p[j] = np.dot(dx(time[j]), sol1)
            dd_p[j] = np.dot(dxdx(time[j]), sol1)
        else:
            if (j * time_step > 2 * Delta and j * time_step <= tf - 2 * Delta):
                p[j] = np.dot(x_l(time[j]), sol2)
                d_p[j] = np.dot([0, 1], sol2)
                dd_p[j] = np.dot([0, 0], sol2)

            else:
                p[j] = np.dot(x(time[j]), sol3)
                d_p[j] = np.dot(dx(time[j]), sol3)
                dd_p[j] = np.dot(dxdx(time[j]), sol3)
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
    for j in range(0, len(time)):
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
    return p_d, dp_d, time


def trajectory_generation(time_step, tf, Delta, Transposition0, Transpositionf):
    # a1, b1, c1 = ikpy.geometry_utils.angles_from_rotation_matrix(Transposition0[:3, :3])
    # a2, b2, c2 = ikpy.geometry_utils.angles_from_rotation_matrix(Transpositionf[:3, :3])
    # theta_desired, r, time = angle_axis_generation(time_step, tf, Delta, np.array([a1, b1, c1]), np.array([a2, b2, c2]))
    theta_desired, dtheta_desired, r, time = angle_axis_generation(time_step, tf, Delta, Transposition0[:3, :3], Transpositionf[:3, :3])
    theta_desired = np.array([theta_desired])
    dtheta_desired = np.array([dtheta_desired])
    p_desired, dp_desired, time = line_generation(time_step, tf, Delta, Transposition0[:3, 3], Transpositionf[:3, 3])
    print(dp_desired.shape)
    print(dtheta_desired.shape)
    transposition_desired = np.concatenate((p_desired, theta_desired))
    dtransposition_desired = np.concatenate((dp_desired, dtheta_desired))
    return transposition_desired, dtransposition_desired, r, time
# p_d, time = Trajectory_Generation(0.1, 10, 1, [0, 0, 0], [10, 10, 10], "line")
# print(p_d)
# print(time)
