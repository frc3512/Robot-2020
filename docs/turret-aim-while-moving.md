Turret aim while moving
=======================

Law of sines:
sin A / a = sin B / b

Cross product of two vectors:
sin A = b x c / (|b| |c|)
sin B = a x c / (|a| |c|)
sin C = a x b / (|a| |b|)

Let g subscript refer to the goal, b subscript refer to the ball, t be the time
of flight of the ball, x be a position vector, v be a velocity vector, and s be
a speed (norm of the velocity vector).

Instead of viewing it as the drivetrain moving and target being stationary, view
it as the target moving and the drivetrain being stationary. Then the only
velocities that matter are the ball and the target where the drivetrain position
is just the starting point. Therefore, v_g = -v_r.

```
       s_g t
      <----- x_g
     ^ A   B |
      \      |
       \     |
 s_b t  \    | d
         \   |
          \  |
           \C|
           x_b
```

Angle A is opposite of side a, angle B is opposite of side b, etc. x_b is the
initial position vector of the ball. We want to find angle C, which is the
turret heading adjustment from a moving target or drivetrain. "x" alone is the
cross product.

Find the equation for sin C.

sin B / s_b t = sin C / s_g t
sin B / s_b = sin C / s_g
sin C = sin B s_g / s_b

Find sin B.

sin B = a x c / (|a| |c|)
sin B = (x_g - x_b) x v_g / (s_g |x_g - x_b|)

Substitute back in to equation for sin C.

sin C = (x_g - x_b) x v_g / (s_g |x_g - x_b|) * s_g / s_b
sin C = (x_g - x_b) x v_g / (|x_g - x_b| s_b)

θ_t in global frame with no movement = atan2(x_g.y - x_r.y, x_g.x - x_r.x)
θ_t in drivetrain frame = θ_t in global - θ_r
θ_t adjustment = C
