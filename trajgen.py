
import numpy as np
import matplotlib.pyplot as plt


def x(t):

    return np.array([1, t, t**2, t**3, t**4, t**5])


def x_l(t):

    return np.array([1, t])


def dx(t):

    return np.array([0, 1, 2 * t, 3 * t**2, 4 * t**3, 5 * t**4])


def dxdx(t):

    return np.array([0, 0, 2, 6 * t, 12 * t**2, 20 * t**3])


def Trajectory_Generation(time_step, tf, Delta, p0, pf, type):

    time = np.arange(0, tf + time_step, time_step)
    global h1
    global h2
    # helix code
    # r = 5
    # alpha = 5
    lamda_x = float((pf[0] - p0[0])) / tf
    lamda_y = float((pf[1] - p0[1])) / tf
    lamda_z = float((pf[2] - p0[2])) / tf

    flag0 = type == 'line'
    flag1 = type == 'helix'
    flag2 = lamda_x == 0

    flag3 = lamda_y == 0
    flag4 = lamda_z == 0
    if flag1 and ((not flag2 and not flag3) or(not flag2 and not flag4) or(not flag3 and not flag4)):
        print('Wrong Parameters for helix generation: ABORTING\n')
        exit(0)
    if not flag0 and not flag1:
        print('Two types of trajectories: line and helix\n')
        exit(0)
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
    # p_d_1 = p
    # p_d_2 = p
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
    for j in range(0, len(time)):
        if flag0:
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

            else:
                p_d[1, j] = lamda_y * (p_d[0, j] - p0[0]) / lamda_x + p0[1]
                p_d[2, j] = lamda_z * (p_d[0, j] - p0[0]) / lamda_x + p0[2]
    print(len(time))
    plt.figure(1)
    plt.plot(time, p, label="trajectory", color='b')
    plt.plot(time, d_p, label="1st derivative", color='y')
    plt.plot(time, dd_p, label="2nd derivative", color='r')
    plt.title('Trajectory with derivatives')
    # plt.legend('trajectory', '1st derivative', '2nd derivative')
    plt.xlabel('time(sec)')
    plt.draw()
    plt.show()
    return p_d, time


p_d, time = Trajectory_Generation(0.1, 10, 1, [0, 0, 0], [10, 10, 10], "line")
# print(p_d)
# print(time)
