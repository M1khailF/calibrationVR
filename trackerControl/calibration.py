import triad_openvr
import numpy as np
import time
import matplotlib.pyplot as plt
from math import atan2, pi
import json

def estimate_coef(x, y):
    n = np.size(x)

    m_x = np.mean(x)
    m_y = np.mean(y)

    SS_xy = np.sum(y*x) - n*m_y*m_x
    SS_xx = np.sum(x*x) - n*m_x*m_x

    b_1 = SS_xy / SS_xx
    b_0 = m_y - b_1*m_x

    return (b_0, b_1)

def plot_regression_line(x, y, b):

    plt.scatter(x, y, color = "m", marker="o", s=30)

    y_pred = b[0] + b[1]*x

    plt.plot(x, y_pred, color = "g")

    plt.show()

def main():
    v = triad_openvr.triad_openvr()

    points_x = []
    points_y = []

    print("Start calibration")
    while True:
        current_point = v.devices["tracker_1"].get_pose_euler()
        if current_point is None:
            continue
        else:
            points_x.append(current_point[0])
            points_y.append(-current_point[2])

        time.sleep(0.1)
        if len(points_x) > 20:
            print("Stop calibration")
            break

    points_x = np.array(points_x)
    points_y = np.array(points_y)

    b = estimate_coef(points_x, points_y)
    # plot_regression_line(points_x, points_y, b)

    x1 = min(points_x)
    x2 = max(points_x)

    y1 = b[0] + b[1]*x1
    y2 = b[0] + b[1]*x2

    x = x2-x1
    y = y2-y1

    angle = atan2(y, x) - pi/2

    with open("z_angle_rotation.json", "w") as f:
        json.dump({'z_angle': angle}, f)

if __name__ == "__main__":
    main()
