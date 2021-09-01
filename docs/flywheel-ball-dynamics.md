# Flywheel ball dynamics

Let `J_f` be the moment of inertia of the flywheel, `J_b` be the moment of
inertia of the ball, `omega_f` be the angular velocity of the flywheel, and
`omega_b` be the angular velocity of the ball.

According the conservation of angular momentum, the flywheel reaches the
following steady-state.

```
angular momenum before = angular momentum after
J_f omega_f + J_b omega_b = (J_f + J_b) omega
J_f omega_f + J_b (0) = (J_f + J_b) omega
J_f omega_f = (J_f + J_b) omega
omega = J_f / (J_f + J_b) omega_f
```

`J_f / (J_f + J_b) = 0.92` from real shooter data (the lowest point in the
flywheel angular velocity drop provides the multiplier).
