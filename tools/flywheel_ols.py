#!/usr/bin/env python3
"""Runs OLS on flywheel velocity recordings generated from step voltage inputs.
"""
import math
import matplotlib.pyplot as plt
import numpy as np

filename = "Flywheel characterization.csv"

# Get labels from first row of file
with open(filename) as f:
    labels = [x.strip('"') for x in f.readline().rstrip().split(",")]

# Retrieve data from remaining rows of file
print(f"Plotting {filename}")
data = np.genfromtxt(filename, delimiter=",", skip_header=1, skip_footer=1)
ts = data[:, 0]
us = data[:, 1]
xs = np.zeros((len(ts), 1))
for i in range(len(ts)):
    xs[i, 0] = data[i, 2]

y = np.zeros((len(ts) - 1, 1))
for i in range(y.shape[0]):
    y[i, 0] = xs[i + 1, 0]

X = np.zeros((len(ts) - 1, 3))
for i in range(X.shape[0]):
    X[i, 0] = xs[i, 0]
    X[i, 1] = us[i]
    X[i, 2] = np.sign(xs[i, 0])

# Calculate b = β that minimizes u'u.
b = np.linalg.solve(X.T @ X, X.T) @ y
alpha = b[0, 0]
beta = b[1, 0]
gamma = b[2, 0]

n = X.shape[0]
sse = np.linalg.norm(y - X @ b, ord=2)
ssto = ((y.T @ y) - (1 / n) * (y.T @ y))[0, 0]
r2 = (ssto - sse) / ssto
adjR2 = 1 - (1 - r2) * ((n - 1.0) / (n - 3))
print(f"R^2={adjR2}")

dts = []
for i in range(1, len(ts)):
    dt = ts[i] - ts[i - 1]
    if dt > 0.0 and dt < 1.0:
        dts.append(dt)
T = sum(dts) / len(dts)
print(f"T = {T}")

Ks = -gamma / beta
Kv = (1.0 - alpha) / beta
Ka = (alpha - 1.0) * T / (beta * math.log(alpha))

print(f"Ks={Ks}")
print(f"Kv={Kv}")
print(f"Ka={Ka}")

# First label is x axis label (time). The remainder are dataset names.
plt.figure()
plt.title("Flywheel inputs")
plt.xlabel("Time (s)")
plt.ylabel("Inputs")
plt.plot(ts, us, label="Input voltage (V)")
plt.legend()

plt.figure()
plt.title("Flywheel outputs")
plt.xlabel("Time (s)")
plt.ylabel("Measurements")
plt.plot(ts, xs, label="Angular velocity measurements (rad/s)")
plt.legend()

plt.figure()
plt.title("Flywheel simulation")
plt.xlabel("Time (s)")
plt.plot(ts, xs, label="Angular velocity measurements (rad/s)")

A = -Kv / Ka
B = 1.0 / Ka
c = -Ks / Ka

pxs = []
pys = []

# Get data list and initialize model state
pxs.append(ts[0])
pys.append(xs[0, 0])

x = xs[0, 0]
t = ts[0]
for j in range(1, xs.shape[0]):
    dt = ts[j] - ts[j - 1]
    t += dt

    # If there's a large gap or the time went backwards, it's a new
    # section of data, so reset the model state
    if dt < 0.0 or dt > 1.0:
        plt.plot(pxs, pys, label="Angular velocity (rad/s)")
        pxs = []
        pys = []
        x = xs[j]
        continue

    # Given dx/dt = Ax + Bu + c,
    # x_k+1 = e^(AT) x_k + A^-1 (e^(AT) - 1) (Bu + c)
    Ad = math.exp(A * dt)
    Bd = 1.0 / A * (Ad - 1) * B
    x = Ad * x + Bd * us[j - 1] + 1.0 / A * (Ad - 1) * c * np.sign(x)
    pxs.append(t)
    pys.append(x)
plt.plot(pxs, pys, label="Angular velocity estimate (rad/s)")
plt.legend()

plt.show()
