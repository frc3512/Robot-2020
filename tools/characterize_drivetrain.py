#!/usr/bin/env python3
"""Uses an Extended Kalman filter to estimate Kv and Ka for a position system
with a constant voltage applied from rest.

Run this script on successive estimates of Kv/Ka until the initial and final
Kv/Ka converge to each other.
"""
from enum import Enum
import frccontrol as fct
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp


class Type(Enum):
    LINEAR = 0
    ANGULAR = 1


UMAX = 4.0
TYPE = Type.ANGULAR

# Initial Kv = voltage applied / last velocity estimate (hopefully it's near
#       steady-state velocity)
# Initial Ka = 1 (arbitrary nonzero value to avoid numerical instability)
KV_INIT = 4.42
KA_INIT = 1.0
DT_INIT = 0.005


def discretize_aq(contA, contQ, dt):
    """Discretizes the given continuous A and Q matrices.

    Keyword arguments:
    contA -- continuous system matrix
    contQ -- continuous process noise covariance matrix
    dt    -- discretization timestep in seconds

    Returns:
    Discretized A and Q.
    """
    # Make continuous Q symmetric if it isn't already
    Q = (contQ + contQ.T) / 2.0

    # Set up the matrix M = [[-A, Q], [0, A.T]]
    STATES = contA.shape[0]
    M = np.zeros((2 * STATES, 2 * STATES))
    M[:STATES, :STATES] = -contA
    M[:STATES:, STATES:] = Q
    M[STATES:, STATES:] = contA.T

    phi = sp.linalg.expm(M * dt)

    # Phi12 = phi[0:States,        States:2*States]
    # Phi22 = phi[States:2*States, States:2*States]
    phi12 = phi[:STATES, STATES:]
    phi22 = phi[STATES:, STATES:]

    discA = phi22.T

    Q = discA @ phi12

    # Make discrete Q symmetric if it isn't already
    return discA, (Q + Q.T) / 2.0


def dynamics(x, u):
    """
    Linear model:
        [x]
    x = [v]

           [0,      1]     [   0]
    xdot = [0, -Kv/Ka] x + [1/Ka] u

    Augmented model:
        [ x]
    x = [ v]
        [Kv]
        [Ka]
    xdot = f(x, u)
    dx/dt = v
    dv/dt = -Kv/Ka v + 1/Ka u
    dKv/dt = 0
    dKa/dt = 0
    """
    v = x[1, 0]
    Kv = x[2, 0]
    Ka = x[3, 0]
    return np.array([[v], [-Kv / Ka * v + 1.0 / Ka * u[0, 0]], [0], [0]])


def jacobian_x(x, u):
    """
    Linear model:
        [x]
    x = [v]

           [0,      1]     [   0]
    xdot = [0, -Kv/Ka] x + [1/Ka] u

    Augmented model:
        [ x]
    x = [ v]
        [Kv]
        [Ka]
    xdot = f(x, u)
    dx/dt = v
    dv/dt = -Kv/Ka v + 1/Ka u
    dKv/dt = 0
    dKa/dt = 0

    third line w/r/t Ka:
    = -Kv * v * -1/Ka**2 + u * -1/Ka**2
    = Kv * v / Ka**2 - u / Ka**2
    = (Kv * v - u) / Ka**2

            [0,      1,     0,                  0]
            [0, -Kv/Ka, -v/Ka, (Kv v - u) / Ka**2]
    df/dx = [0,      0,     0,                  0]
            [0,      0,     0,                  0]
    """
    v = x[1, 0]
    Kv = x[2, 0]
    Ka = x[3, 0]
    return np.array([
        [0, 1, 0, 0],
        [0, -Kv / Ka, -v / Ka, (Kv * v - u[0, 0]) / Ka**2],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ])


if TYPE == Type.LINEAR:
    filename = "drivetrainLinearMeasurements.txt"
elif TYPE == Type.ANGULAR:
    filename = "turretAngularMeasurements.txt"

# Get labels from first row of file
with open(filename) as f:
    labels = [x.strip('"') for x in f.readline().rstrip().split(",")]

# Retrieve data from remaining rows of file
print(f"Plotting {filename}")
data = np.genfromtxt(filename, delimiter=",", skip_header=1, skip_footer=1)
ts = data[:, 0]
# us = data[:, 1]
us = [UMAX] * len(ts)
ys = np.zeros((len(ts), 1))
for i in range(len(ts)):
    if TYPE == Type.LINEAR:
        ys[i, 0] = (data[i, 2] + data[i, 3]) / 2.0
    elif TYPE == Type.ANGULAR:
        ys[i, 0] = data[i, 1]

# First label is x axis label (time). The remainder are dataset names.
plt.figure()
plt.title("Drivetrain measurements")
plt.xlabel("Time (s)")
plt.ylabel("Measurements")
plt.plot(ts, ys, label="Position (m)")

vels = [0]
for i in range(1, len(ys)):
    vels.append((ys[i] - ys[i - 1]) / (ts[i] - ts[i - 1]))
plt.plot(ts, vels, label="Velocity (numerical derivative of position) (m/s)")

accels = [0]
for i in range(1, len(ys)):
    accels.append((vels[i] - vels[i - 1]) / (ts[i] - ts[i - 1]))
# plt.plot(ts, accels, label="Acceleration (m/s^2)")

plt.legend()

# Measuring position state
C = np.array([[1, 0, 0, 0]])

# Have full certainty of initial position and velocity, but little certainty of
# Kv and Ka
P = np.diag([0, 0, 0, 10])

# The model has no uncertainty for position or velocity, so their process noise
# covariance is set to zero. We're only estimating Kv and Ka states
Q = np.diag(np.square([0, 0, 0, 10]))

R = np.diag(np.square([0.00001]))

num_points = len(us)

# Initial Kv = voltage applied / last velocity estimate (hopefully it's near
#       steady-state velocity)
# Initial Ka = 1 (arbitrary nonzero value to avoid numerical instability)
while True:
    xhat = np.array([[2.5], [0], [KV_INIT], [KA_INIT]])
    xhat_rec = np.zeros((4, 1, num_points))
    P_rec = np.zeros((4, 4, num_points))

    for k in range(num_points):
        if k == 0:
            xhat_rec[:, :, k] = xhat
            P_rec[:, :, k] = P
        else:
            u = np.array([[us[k]]])
            y = np.array([ys[k]])

            # Predict
            A = jacobian_x(xhat, u)
            if k > 0:
                xhat = fct.runge_kutta(dynamics, xhat, u, ts[k] - ts[k - 1])
                discA, discQ = discretize_aq(A, Q, ts[k] - ts[k - 1])
            else:
                xhat = fct.runge_kutta(dynamics, xhat, u, DT_INIT)
                discA, discQ = discretize_aq(A, Q, DT_INIT)

            P = discA @ P @ discA.T + discQ

            # Update
            discR = R / ts[k]
            K = P @ C.T @ np.linalg.inv(C @ P @ C.T + discR)
            xhat += K @ (y - C @ xhat)
            P = (np.eye(4, 4) - K @ C) @ P

            # Clamp Kv and Ka to positive values
            if xhat[2, 0] < 0.0:
                xhat[2, 0] = KV_INIT
            if xhat[3, 0] < 0.0:
                xhat[3, 0] = KA_INIT

            xhat_rec[:, :, k] = xhat
            P_rec[:, :, k] = np.array([P[0, 0], P[1, 1], P[2, 2], P[3, 3]]).T

    break
    if abs(KV_INIT - xhat[2, 0]) < 1e-5 and abs(KA_INIT - xhat[3, 0]) < 1e-5:
        break
    KV_INIT = xhat[2, 0]
    KA_INIT = xhat[3, 0]
print("E[Kv]=", xhat[2, 0])
print("E[Ka]=", xhat[3, 0])

plt.figure()
plt.title("Kv and Ka estimate over time")
plt.xlabel("Time (s)")
plt.plot(ts, xhat_rec[0, 0, :], label="Position estimate")
plt.plot(ts, ys, label="Position measurement")
plt.plot(ts, xhat_rec[1, 0, :], label="Velocity estimate")
plt.plot(ts, vels, label="Velocity measurement")
plt.plot(ts, xhat_rec[2, 0, :], label="Kv estimate")
plt.plot(ts, xhat_rec[3, 0, :], label="Ka estimate")
plt.legend()

plt.figure()
plt.title("EKF variances")
plt.xlabel("Time (s)")
plt.plot(ts, P_rec[0, 0, :], label="Position variance (m^2)")
plt.plot(ts, P_rec[1, 1, :], label="Velocity variance (m^2/s^2)")
plt.plot(ts, P_rec[2, 2, :], label="Kv variance ((V/m/s)^2)")
plt.plot(ts, P_rec[3, 3, :], label="Ka variance ((V/m/s^2)^2)")
plt.legend()

xhat = np.array([[0], [0], [xhat[2, 0]], [xhat[3, 0]]])
xhat_rec = np.zeros((4, 1, num_points))

for k in range(num_points):
    if k == 0:
        xhat_rec[:, :, k] = xhat
    else:
        u = np.array([[us[k]]])

        if k > 0:
            xhat = fct.runge_kutta(dynamics, xhat, u, ts[k] - ts[k - 1])
        else:
            xhat = fct.runge_kutta(dynamics, xhat, u, DT_INIT)
        xhat_rec[:, :, k] = xhat

plt.figure()
plt.title("EKF replay with Kv and Ka constants")
plt.xlabel("Time (s)")
plt.plot(ts, xhat_rec[0, 0, :], label="Position estimate")
plt.plot(ts, ys, label="Position measurement")
plt.plot(ts, xhat_rec[1, 0, :], label="Velocity estimate")
plt.plot(ts, vels, label="Velocity measurement")
plt.plot(ts, xhat_rec[2, 0, :], label="Kv estimate")
plt.plot(ts, xhat_rec[3, 0, :], label="Ka estimate")
plt.legend()

plt.show()
