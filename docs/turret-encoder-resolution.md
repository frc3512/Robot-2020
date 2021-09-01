# Turret encoder resolution

"Rollover" is when the absolute encoder passes through the end of its range and
appears on the other side (the reading resets). We can detect that occurence at
runtime to keep track of more than one encoder revolution worth of turret
travel, but not across robot reboots. The 160:18 reduction between the encoder
and turret means `360 encoder° * 18 turret°/160 encoder° = 40.5 turret°` before
rollover. If the robot starts more than one encoder revolution away from
dead-center (40.5 turret°), we have no way of knowing which 40.5° range the
turret is in. The 10:1 reduction makes the turret rotate 10 times as far in 1
encoder revolution, so the valid turret start range increases from 40.5° to
405°.

The downside is the turret measurement resolution drops by 10x because the same
encoder resolution maps to 10x the turret movement. The
[encoder datasheet](https://www.revrobotics.com/content/docs/REV-11-1271-DS.pdf)
says it has 10-bit resolution, so there's 2^10 possible values to represent 360
encoder°. Therefore, `360 encoder° / 2^10 values = 0.35 encoder°/LSB`.
([LSB](https://en.wikipedia.org/wiki/Bit_numbering#Least_significant_bit) means
least significant bit, so our encoder reading changes 0.35° for each increment
of the least significant bit.) The 160:18 reduction gives `0.35 encoder°/LSB *
18 turret°/160 encoder° = 0.03955 turret°/LSB`. The 10:1 reduction decreases the
resolution to 0.3955 turret°/LSB, which is still adequate. The best we could do
for resolution is a 240° range with a reduction of 240 / 40.5 = 5.926:1, which
means a resolution of 0.234 turret°/LSB. The 240° range is special because
that's how far the turret can move right now without hitting hard stops.
