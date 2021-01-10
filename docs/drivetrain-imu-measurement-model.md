# Drivetrain IMU measurement model

## Longitudinal acceleration

a_long = (a_l + a_r) / 2

## Lateral acceleration

a_lat = v^2 / r

v = wr
r = v/w

a_lat = v^2 / (v/w)
a_lat = v^2 * w/v
a_lat = v * w
a_lat = (v_r + v_l) / 2 * (v_r - v_l) / (2r_b)
a_lat = (v_r + v_l)(v_r - v_l) / (4r_b)
a_lat = (v_r^2 - v_l^2) / (4r_b)

## Model

x = [[x],
     [y],
     [heading],
     [left vel],
     [right vel],
     [left pos],
     [right pos]]

y = [[heading],
     [left pos],
     [right pos],
     [longitudinal acceleration]]

C = [[0, 0, 1, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 1, 0],
     [0, 0, 0, 0, 0, 0, 1],
     [0, 0, 0, 0, 0, 0, 0]]

C[3, 0:6] = 0.5 * (A[3, 0:6] + A[4, 0:6])

D = [[0, 0],
     [0, 0],
     [0.5 * (B[3, 0] + B[4, 0]), 0.5 * (B[3, 1] + B[4, 1])]]
