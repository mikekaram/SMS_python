
import numpy as np


def x(t):

    return [1, t, t**2, t**3, t**4, t**5]


def x_l(t):

    return [1, t]


def dx(t):

    return [0, 1, 2 * t, 3 * t**2, 4 * t**3, 5 * t**4]


def dxdx(t):

    return [0, 0, 2, 6 * t, 12 * t**2, 20 * t**3]


def Trajectory_Generation(time_step, tf, Delta, p0, pf, type):

    # function[p_d, time] = Trajectory_Generation(time_step, tf, Delta, p0, pf, type)
    time = np.arange(0, tf, time_step)
    global h1
    global h2
    # helix code
    r = 5
    alpha = 5
    lamda_x = (pf[0] - p0[0]) / tf
    lamda_y = (pf[1] - p0[1]) / tf
    lamda_z = (pf[2] - p0[2]) / tf
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

    if(Delta >= .5 * (tf - time[0])):
        print('The input you gave is not valid')
        exit(0)
    dp = float((p_final - p_zero)) / (tf - 2 * Delta)
    pf1 = p_final - dp * Delta
    p02 = p_zero + dp * Delta
    A1 = np.array([x(0)], [x(2 * Delta)], [dx(0)], [dx(2 * Delta)], [dxdx(0)], [dxdx(2 * Delta)])
    B1 = np.transpose(np.array([p_zero, p02, 0, dp, 0, 0]))
    sol1 = np.linalg.solve(A1, B1)
    A2 = np.array([x_l(2 * Delta)], [0, 1])
    B2 = np.transpose(np.array([p02, dp])
    sol1=np.linalg.solve(A2, B2)
    A3=[x(tf - 2 * Delta)x(tf)dx(tf - 2 * Delta)dx(tf)dxdx(tf - 2 * Delta)dxdx(tf)]
    B3=[pf1  p_final dp  0 0 0]'
    sol3=A3\B3

    p=zeros(1, length(time))
    d_p=p
    dd_p=p
    p_d_1=p
    p_d_2=p
    dp_d=zeros(3, length(time))
    ddp_d=zeros(3, length(time))
    for j=1:
        length(time)
        if (j * time_step <= 2 * Delta)
            p(j)=x(time(j)) * sol1
            d_p(j)=dx(time(j)) * sol1
            dd_p(j)=dxdx(time(j)) * sol1
        else:
            if(j * time_step > 2 * Delta & & j * time_step <= tf - 2 * Delta)
            p(j)=x_l(time(j)) * sol2
            d_p(j)=[0 1] * sol2
            dd_p(j)=zeros(1, 2) * sol2
        else:
            p(j)=x(time(j)) * sol3
            d_p(j)=dx(time(j)) * sol3
            dd_p(j)=dxdx(time(j)) * sol3

    if flag0
        if flag2
            x_lim=[-Inf Inf]
        else:
            x_lim=[min(p0(1), pf(1)) max(p0(1), pf(1))]

        if flag3
            y_lim=[-Inf Inf]
        else:
            y_lim=[min(p0[1], pf(2)) max(p0[1], pf(2))]

        if flag4
            z_lim=[-Inf Inf]
        else:
            z_lim=[min(p0(3), pf(3)) max(p0(3), pf(3))]

    else:
        if flag1
        if flag2
            x_lim=[p0(1) - r p0(1) + r]
        else:
            x_lim=[min(p0(1), pf(1)) max(p0(1), pf(1))]

        if flag3
            y_lim=[p0(2) - r p0(2) + r]
        else:
            y_lim=[min(p0(2), pf(2)) max(p0(2), pf(2))]

        if flag4
            z_lim=[p0(3) - r p0(3) + r]
        else:
            z_lim=[min(p0(3), pf(3)) max(p0(3), pf(3))]

    if flag2
        if flag3
            p_d(3, : ) = p
            dp_d(3, : ) = d_p
            ddp_d(3, : ) = dd_p
        else:
            p_d(2, : ) = p
            dp_d(2, : ) = d_p
            ddp_d(2, : ) = dd_p

    else:
        p_d(1, : ) = p
        dp_d(1, : ) = d_p
        ddp_d(1, : ) = dd_p

    for j=1:
        length(time)
        if flag0
            % draw 3D line:
            if flag2
                if flag3
                    p_d(1, j)=p0(1)
                    p_d(2, j)=p0(2)
                else:
                    p_d(1, j)=p0(1)
                    if flag4
                        p_d(3, j)=p0(3)
                    else:
                        p_d(3, j)=lamda_z * (p_d(2, j) - p0(2)) / lamda_y + p0(3)

            else:
                p_d(2, j)=lamda_y * (p_d(1, j) - p0(1)) / lamda_x + p0(2)
                p_d(3, j)=lamda_z * (p_d(1, j) - p0(1)) / lamda_x + p0(3)

        #  else:
        #      % draw 3D helix:
        #      p_d_1(j) = r*cosd(360/alpha*p(j))
        #      p_d_2(j) = r*sind(360/alpha*p(j))
        #      if flag2
        #          if flag3
        #              p_d(1,j) = p_d_1(j)+p0(1)
        #              p_d(2,j) = p_d_2(j)+p0(2)
        #          else:
        #              p_d(1,j) = p_d_1(j)+p0(1)
        #              p_d(3,j)= p_d_2(j)+p0(3)

        #      else:
        #          p_d(2,j) = p_d_1(j)+p0(2)
        #          p_d(3,j) = p_d_2(j)+p0(3)

        # h1 = figure(1)
        #  if(mod(j,5) == 0)
        #      plot3(p_d(1,j),p_d(2,j),p_d(3,j),'-.ob')
        #      hold on
        #      axis([x_lim y_lim z_lim])
        #      title('3D Trajectory Generated')
        #      xlabel('x(cm)')
        #      ylabel('y(cm)')
        #      zlabel('z(cm)')
        #      drawnow limitrate

    hold off
    h2=figure(2)
    title('Trajectory with derivatives')
    plot(time, p)
    hold on
    plot(time, d_p)
    hold on
    plot(time, dd_p)
    leg('trajectory', '1st derivative', '2nd derivative')
    xlabel('time(sec)')
    hold off
