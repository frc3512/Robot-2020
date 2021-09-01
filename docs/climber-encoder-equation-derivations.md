# Climber encoder equation derivations

Because the dyneema wraps around the climber drum, the diameter changes as the
climber moves. Our dyneema has a 0.125 inch diameter. The drum is so small on
the climber, that it isn't negligible like our previous elevators. The drum
diameter is 1 inch when the elevator is at the top and it's about 1.7 inches at
the bottom. At the top, the dyneema is completely unspooled.

## Data collection

Distances (m):

28.375" bottom, 65.75" top

Encoder measurements:

0.002659 bottom, -0.70968 top

Distance difference:

48.375" -> 1.2287 m

Encoder measurement difference:

(0.002659 - -0.70968) = 0.712339 m

0.712339 m / (pi * 1 in * 1/20) = 178.539 rotations

Find the true drum diameter.

```
178.539 * pi * d * 1/20 = 1.2287
d = 1.2287 * 20 / pi / 178.539
d = 0.0438119 m -> 1.7249 in
```

The climber drum has the largest diameter when fully retracted (at the bottom).

| Distance (m) | Drum diameter (in) | Drum diameter (m) |
|--------------|--------------------|-------------------|
|            0 |             1.7249 |           0.04381 |
|       1.2287 |                  1 |            0.0254 |

Let `x` be distance and `y` be drum diameter.

```
y - y_1 = m (x - x_1)
y - 0.04381 = ((0.0254 - 0.04381) / (1.2287 - 0)) (x - 0)
y - 0.04381 = ((0.0254 - 0.04381) / 1.2287) x
y = -0.014983 x + 0.04381
y = 0.04381 - 0.014983 x
```

drum diameter = 0.04381 - 0.014983 distance

## Encoder distance equation

`r` is rotations of the motor.

```
x = pi d_drum G r
x = pi G r d_drum

d_drum = 0.04381 - 0.014983 x

x = pi G r (0.04381 - 0.014983 x)
x = 0.04381 pi G r - 0.014983 pi G r x
(1 + 0.014983 pi G r) x = 0.04381 pi G r
x = 0.04381 pi G r / (1 + 0.014983 pi G r)
```
