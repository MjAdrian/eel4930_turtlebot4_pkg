"""
    Author: Aditya Ramesh
    Determines circular target pose from its ellipse projection onto camera's image.
    
    Sources:
        https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=71362&tag=1 
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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
    # from https://en.wikipedia.org/wiki/Ellipse#General_ellipse but using convention from paper above
    a = major_axis_length**2 * np.sin(ellipse_angle)**2 + minor_axis_length**2 * np.cos(ellipse_angle)**2
    b = 2 * (minor_axis_length**2 - major_axis_length**2) * np.sin(ellipse_angle) * np.cos(ellipse_angle)
    c = major_axis_length**2 * np.cos(ellipse_angle)**2 + minor_axis_length**2 * np.sin(ellipse_angle)**2
    d = -2 * a * ellipse_center_x - b * ellipse_center_y
    e = -b * ellipse_center_x - 2 * c * ellipse_center_y
    f = a * ellipse_center_x**2 + b * ellipse_center_x * ellipse_center_y + c * ellipse_center_y**2 - (major_axis_length**2) * (minor_axis_length**2)

    # Find focal length - In true pinhole cameras, fx should equal fy. It is not exactly the same so take the average
    f0 = (K[0][0] + K[1][1]) / 2

    # Find general ellipse equation coefficient in terms of image ellipse (a-f) and the focal length (fo)
    A = a * f0**2
    B = b * f0**2
    C = c * f0**2
    D = d * f0
    E = e * f0
    F = f

    # Check whether actual values are correct
    # coefficients = [a, b, c, d, e, f]
    # actual_value = [ellipse_center_x, ellipse_center_y, major_axis_length, minor_axis_length, ellipse_angle]
    # check = check_general_ellipse_coefficients(coefficients, actual_value)

    # Find Quadratic form Q
    Q = np.array([
        [A, C/2, D/2],
        [C/2, B, E/2],
        [D/2, E/2, F]
    ])

    # Find eigenvalues and eigenvectors from Q
    eigenvalues, eigenvectors = np.linalg.eig(Q)
    
    # Find P - Rotation matrix
    P = np.empty((3,3))
    P_eigenvalues = np.empty(3)

    # Find e3 and λ3 first
    eigenvalue_sign = np.sign(eigenvalues)
    if (eigenvalue_sign.sum() == 1):
        eig3_ind = np.argmin(eigenvalue_sign)
    else:
        eig3_ind = np.argmax(eigenvalue_sign)
    
    P_eigenvalues[2] = eigenvalues[eig3_ind]
    if (np.dot(eigenvectors[eig3_ind], np.array([0, 0, 1]).T) > 0):
        P[2] = eigenvectors[eig3_ind]
    else:
        P[2] = -eigenvectors[eig3_ind]

    # Drop the selected eigenvector and eigenvalue from list
    eigenvalues = np.delete(eigenvalues, eig3_ind, axis=0)
    eigenvectors = np.delete(eigenvectors, eig3_ind, axis=0)

    # Find e2 and λ2 from the remaining values
    eig2_ind = np.argmin(eigenvalues)
    P[1] = eigenvectors[eig2_ind]
    P_eigenvalues[1] = eigenvalues[eig2_ind]

    # Drop the selected eigenvector and eigenvalue from list
    eigenvalues = np.delete(eigenvalues, eig2_ind, axis=0)
    eigenvectors = np.delete(eigenvectors, eig2_ind, axis=0)

    # Find e1 and λ1 from the remaining values
    P[0] = np.cross(P[1], P[2])
    P_eigenvalues[0] = eigenvalues[0]

    # Find kx, ky, kz from λ1, λ2, λ3
    kx = np.sqrt(1 / np.abs(P_eigenvalues[0]))
    ky = np.sqrt(1 / np.abs(P_eigenvalues[1]))
    kz = np.sqrt(1 / np.abs(P_eigenvalues[2]))

    # Find circle center - notation from paper above 
    lambda_1 = np.abs(P_eigenvalues[0])
    lambda_2 = np.abs(P_eigenvalues[1])
    lambda_3 = np.abs(P_eigenvalues[2])
    e1x, e1y, e1z = P[0]
    e2x, e2y, e2z = P[1]
    e3x, e3y, e3z = P[2]
    R = circle_radius

    c1 = R * np.sqrt((lambda_3 * (lambda_1 - lambda_2)) / (lambda_1 * (lambda_1 + lambda_3)))   # only needs to be calculated once
    c2 = R * np.sqrt((lambda_1 * (lambda_2 + lambda_3)) / (lambda_3 * (lambda_1 + lambda_3)))

    # Solution 1
    circle_x_1 = e1x * c1 + e3x * c2
    circle_y_1 = e1y * c1 + e3y * c2
    circle_z_1 = e1z * c1 + e3z * c2

    # Solution 2
    circle_x_2 = -e1x * c1 + e3x * c2
    circle_y_2 = -e1y * c1 + e3y * c2
    circle_z_2 = -e1z * c1 + e3z * c2

    # Find circle normal - notation from paper above 
    c3 = np.sqrt((lambda_1 - lambda_2) / (lambda_1 + lambda_3))
    c4 = np.sqrt((lambda_2 + lambda_3) / (lambda_1 + lambda_3))

    # Solution 1
    circle_norm_x_1 = e1x * c3 - e3x * c4
    circle_norm_y_1 = e1y * c3 - e3y * c4
    circle_norm_z_1 = e1z * c3 - e3z * c4

    # Solution 2
    circle_norm_x_2 = -e1x * c3 - e3x * c4
    circle_norm_y_2 = -e1y * c3 - e3y * c4
    circle_norm_z_2 = -e1z * c3 - e3z * c4

    return np.array([[[circle_x_1, circle_y_1, circle_z_1], [circle_norm_x_1, circle_norm_y_1, circle_norm_z_1]], 
                     [[circle_x_2, circle_y_2, circle_z_2], [circle_norm_x_2, circle_norm_y_2, circle_norm_z_2]]])

def check_general_ellipse_coefficients(coefficients, actual_value):
    """Checks if the calculated coefficients are the correct values"""
    a, b, c, d, e, f = coefficients
    ellipse_center_x, ellipse_center_y, major_axis_length, minor_axis_length, ellipse_angle = actual_value

    calc_major_axis_length = -np.sqrt(2 * (a*e**2 + c*d**2 - b*d*e + (b**2 - 4*a*c)*f) * ((a+c) + np.sqrt((a-c)**2 + b**2))) / (b**2 - 4*a*c)
    calc_minor_axis_length = -np.sqrt(2 * (a*e**2 + c*d**2 - b*d*e + (b**2 - 4*a*c)*f) * ((a+c) - np.sqrt((a-c)**2 + b**2))) / (b**2 - 4*a*c)
    calc_ellipse_center_x  = (2*c*d - b*e) / (b**2 - 4*a*c)
    calc_ellipse_center_y  = (2*a*e - b*d) / (b**2 - 4*a*c)
    if (b != 0):
        calc_ellipse_angle = np.arctan2(c-a-np.sqrt((a-c)**2 + b**2), b)
    else:
        if (a < c):
            calc_ellipse_angle = 0
        else:
            calc_ellipse_angle = np.pi / 2
    
    # Print comparison
    print('Major Axis:', major_axis_length, calc_major_axis_length)
    print('Minor Axis:', minor_axis_length, calc_minor_axis_length)
    print('Ellipse x:', ellipse_center_x, calc_ellipse_center_x)
    print('Ellipse y:', ellipse_center_y, calc_ellipse_center_y)
    print('Ellipse angle:', ellipse_angle, calc_ellipse_angle, '\n')

    epsilon = 0.0001
    check = (major_axis_length - calc_major_axis_length < epsilon
            and minor_axis_length - calc_minor_axis_length < epsilon
            and ellipse_center_x - calc_ellipse_center_x < epsilon
            and ellipse_center_y - calc_ellipse_center_y < epsilon
            and ellipse_angle - calc_ellipse_angle < epsilon)
    
    if (check):
        print('Coefficients are correct')
    else:
        print('Coefficient are not correct')

    return check 

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
    plt.pause(1/60)
    plt.draw()

def plot_circle_norm_3d(circles, ax):
    """Visualizes the normal of circle in 3D"""
    alpha = [1, 0.5]
    color = ['b', 'r']
    for i, circle in enumerate(circles):
        ax.quiver(circle[0][0], circle[0][1], circle[0][2], circle[1][0], circle[1][1], circle[1][2], alpha=alpha[i], color=color[i])
        ax.set_xlabel('X'), ax.set_ylabel('Y'), ax.set_zlabel('Z')
        ax.set_xlim(-2.5, 2.5), ax.set_ylim(-2.5, 2.5), ax.set_zlim(-2.5, 2.5)
    plt.pause(1/60)
    ax.cla()