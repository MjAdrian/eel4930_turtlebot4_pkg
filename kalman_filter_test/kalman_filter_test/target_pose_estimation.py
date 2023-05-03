"""
    Author: Aditya Ramesh
    Determines circular target pose from its ellipse projection onto camera's image.
    
    Sources:
        https://www.eecg.toronto.edu/~pagiamt/kcsmith/00163786.pdf
"""

import numpy as np
import matplotlib.pyplot as plt

def determine_target_pose(K: np.ndarray, ellipse: np.ndarray, circle_radius = 2):
    """Determines and returns taget pose in 3D

        Parameters:
            K               - Intrinsic parameter of camera
            ellipse         - detected projected ellipse
            circle_radius   - radius of circle (in inches)
    
        Returns two potential solutions of ellipse pose
            [[x,y,z], [x_norm, y_norm, z_norm]]
    """
    # ellipse = [(x center, y center), (major axis, minor axis), angle]
    ellipse_center_x, ellipse_center_y = ellipse[0]             # (x center, y center)
    ellipse_axis = ellipse[1]               # [major axis, minor axis]
    ellipse_angle = np.deg2rad(-ellipse[2]) # angles are reversed in openCV

    major_axis_length = np.max(ellipse_axis) # major axis length
    minor_axis_length = np.min(ellipse_axis) # minor axis length

    # Find general ellipse equation coefficients
    # from https://en.wikipedia.org/wiki/Ellipse#General_ellipse
    A = major_axis_length**2 * np.sin(ellipse_angle)**2 + minor_axis_length**2 * np.cos(ellipse_angle)**2
    B = 2 * (minor_axis_length**2 - major_axis_length**2) * np.sin(ellipse_angle) * np.cos(ellipse_angle)
    C = major_axis_length**2 * np.cos(ellipse_angle)**2 + minor_axis_length**2 * np.sin(ellipse_angle)**2
    D = -2 * A * ellipse_center_x - B * ellipse_center_y
    E = -B * ellipse_center_x - 2 * C * ellipse_center_y
    F = A * ellipse_center_x**2 + B * ellipse_center_x * ellipse_center_y + C * ellipse_center_y**2 - (major_axis_length**2) * (minor_axis_length**2)

    # Find coefficient for ellipse equation defined in paper from above coefficients
    a_prime = A
    b_prime = C
    d_prime = F
    f_prime = E / 2
    g_prime = D / 2
    h_prime = B / 2

    # Find focal length - In true pinhole cameras, fx should equal fy. It is not exactly the same so take the average
    f0 = (K[0][0] + K[1][1]) / 2

    # Vertex points (x, y, z) = (α, β, γ)
    alpha = 0
    beta = 0
    gamma = -f0

    # Find coefficient of cone
    a = gamma**2 * a_prime
    b = gamma**2 * b_prime
    c = a_prime * alpha**2 + 2*h_prime*alpha*beta + b_prime*beta**2 + 2*g_prime*alpha + 2*f_prime*beta + d_prime
    d = gamma**2 * d_prime
    f = -gamma * (b_prime*beta + h_prime*alpha + f_prime)
    g = -gamma * (h_prime*beta + a_prime*alpha + g_prime)
    h = gamma**2 * h_prime
    u = gamma**2 * g_prime
    v = gamma**2 * f_prime
    w = -gamma * (f_prime*beta + g_prime*alpha + d_prime)

    # Find roots to determine λ
    cubic_a = 1; 
    cubic_b = -(a + b + c) 
    cubic_c = b*c + c*a + a*b - f**2 - g**2 - h**2
    cubic_d = -(a*b*c + 2*f*g*h - a*f**2 - b*g**2 - c*h**2)
    lambda_1, lambda_2, lambda_3 = np.roots([cubic_a, cubic_b, cubic_c, cubic_d])
    
    # Estimation of the coefficients of the equation of the circular-feature plane - 2 solutions
    if (lambda_1 < lambda_2):
        n_1 = np.sqrt((lambda_1 - lambda_3) / (lambda_2 - lambda_3))
        m_1 = np.sqrt((lambda_2 - lambda_1) / (lambda_2 - lambda_3))
        l_1 = 0 

        n_2 = np.sqrt((lambda_1 - lambda_3) / (lambda_2 - lambda_3))
        m_2 = -np.sqrt((lambda_2 - lambda_1) / (lambda_2 - lambda_3))
        l_2 = 0 
    elif (lambda_1 > lambda_2):
        n_1 = np.sqrt((lambda_2 - lambda_3) / (lambda_1 - lambda_3))
        m_1 = 0
        l_1 = np.sqrt((lambda_1 - lambda_2) / (lambda_1 - lambda_3))

        n_2 = np.sqrt((lambda_2 - lambda_3) / (lambda_1 - lambda_3))
        m_2 = 0
        l_2 = -np.sqrt((lambda_1 - lambda_2) / (lambda_1 - lambda_3))
    else: # (lambda_1 == lambda_2)
        n_1 = 1
        m_1 = 0
        l_1 = 0

        n_2 = 1
        m_2 = 0
        l_2 = 0

    vec_1 = np.matrix([l_1, m_1, n_1, 1]).T
    vec_2 = np.matrix([l_2, m_2, n_2, 1]).T

    # Find transformation matrix
    t1 = np.array([(b - lambda_1) * g - f*h,
                   (b - lambda_2) * g - f*h,
                   (b - lambda_3) * g - f*h])
    t2 = np.array([(a - lambda_1) * f - g*h,
                   (a - lambda_2) * f - g*h,
                   (a - lambda_3) * f - g*h])
    t3 = np.array([-(a - lambda_1) * (t1[0]/t2[0]) / g - (h/g),
                   -(a - lambda_2) * (t1[1]/t2[1]) / g - (h/g),
                   -(a - lambda_3) * (t1[2]/t2[2]) / g - (h/g)])
    
    m_1 = 1 / (np.sqrt(1 + (t1[0]/t2[0])**2 + t3[0]**2))
    m_2 = 1 / (np.sqrt(1 + (t1[1]/t2[1])**2 + t3[1]**2))
    m_3 = 1 / (np.sqrt(1 + (t1[2]/t2[2])**2 + t3[2]**2))

    l_1 = (t1[0]/t2[0]) * m_1
    l_2 = (t1[1]/t2[1]) * m_2
    l_3 = (t1[2]/t2[2]) * m_3

    n_1 = t3[0] * m_1
    n_2 = t3[1] * m_2
    n_3 = t3[2] * m_3

    T = np.matrix([[l_1, l_2, l_3, 0],
                   [m_1, m_2, m_3, 0],
                   [n_1, n_2, n_3, 0],
                   [0, 0, 0, 1]])

    # Find orientations
    circle_norm_1 = T @ vec_1
    circle_norm_2 = T @ vec_2

    circle_norm_1 = np.asarray(circle_norm_1).flatten().tolist()
    circle_norm_2 = np.asarray(circle_norm_2).flatten().tolist()

    # Find circle coefficients
    A = lambda_1*l_1**2 + lambda_2*l_2**2 + lambda_3*l_3**2
    B = lambda_1*l_1*n_1 + lambda_2*l_2*n_2 + lambda_3*l_3*n_3
    C = lambda_1*m_1*n_1 + lambda_2*m_2*n_2 + lambda_3*m_3*n_3
    D = lambda_1*n_1**2 + lambda_2*n_2**2 + lambda_3*n_3**2
    r = circle_radius

    # Find position
    Z = A*r / (np.sqrt(np.abs(B*B + C*C - A*D)))
    X = -B/A * Z
    Y = -C/A * Z

    # print(X,Y,Z)
    
    return np.array([[[X, Y, Z], [circle_norm_1[0], circle_norm_1[1], circle_norm_1[2]]],
                     [[X, Y, Z], [circle_norm_2[0], circle_norm_2[1], circle_norm_2[2]]]])


def initialize_2d_plot():
    """Initialize 2D plot for plot_ellipse_norm_2d"""
    fig_2d = plt.figure()
    ax_2d_x = fig_2d.add_subplot(311)
    ax_2d_y = fig_2d.add_subplot(312)
    ax_2d_z = fig_2d.add_subplot(313)
    ax_2d = np.array([ax_2d_x, ax_2d_y, ax_2d_z])
    x_label = 'time'
    y_label = ['X', 'Y', 'Z']
    for i, axis in enumerate(ax_2d):
        axis.set_xlabel(x_label), axis.set_ylabel(y_label[i])
        axis.set_ylim(-1, 1)
    plt.ion()
    return fig_2d, ax_2d

def initalize_3d_plot():
    """Initialize 3D plot for plot_ellipse_norm_3d"""
    fig_3d = plt.figure()
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    plt.ion()
    return fig_3d, ax_3d

def plot_circle_norm_2d(circle_normal, fig, ax, time, time_lim = 500):
    """Visualizes the normal of the circle in 2D"""
    color = ['.-r', '.-g', '.-b']
    for i, axis in enumerate(ax):
        axis.plot(time % time_lim, circle_normal[i], color[i])
    if (time % time_lim == 0):
        ax[0].cla()
        ax[1].cla()
        ax[2].cla()
    plt.pause(1/30)
    plt.draw()

def plot_circle_norm_3d(circle_pose, ax, axis_lim=(-20,20)):
    """Visualizes the normal of circle in 3D"""
    size = 25
    ax.quiver(circle_pose[0], circle_pose[1], circle_pose[2], size*circle_pose[3], size*circle_pose[4], size*circle_pose[5], alpha=1, color='b')
    ax.set_xlabel('X'), ax.set_ylabel('Y'), ax.set_zlabel('Z')
    ax.set_xlim(axis_lim), ax.set_ylim(axis_lim), ax.set_zlim(axis_lim)
    plt.pause(1/30)
    ax.cla()

def plot_circle_norm_3d_both_solutions(circles, ax, axis_lim=(-50,50)):
    """Visualizes the normal of circle in 3D"""
    alpha = [1, 0.5]
    color = ['b', 'r']
    size = 10
    for i, circle in enumerate(circles):
        ax.quiver(circle[0][0], circle[0][1], circle[0][2], size*circle[1][0], size*circle[1][1], size*circle[1][2], alpha=alpha[i], color=color[i])
        ax.set_xlabel('X'), ax.set_ylabel('Y'), ax.set_zlabel('Z')
        ax.set_xlim(axis_lim), ax.set_ylim(axis_lim), ax.set_zlim(axis_lim)
    plt.pause(1/30)
    ax.cla()