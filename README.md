# FRC team 3512's 2020 robot

Source code for the 2020 comp robot: Arcadia

Source code also for the 2020 practice robot: Ellinika

## Setup

Install the relevant FRC toolchain for your platform
(see https://github.com/wpilibsuite/allwpilib/releases). Make sure
`~/frc2020/roborio/bin` is in PATH.

Install the following OS packages.

* gcc >= 7.3.0
* python >= 3.6

Install the following python packages via `pip3 install --user package_name`.

* wpiformat (optional)
  * https://github.com/wpilibsuite/styleguide/blob/master/wpiformat/README.rst

We use clang-format 9.0 with wpiformat for C++ formatting.

## Build

* `./gradlew-build-athena.sh`

This runs a roboRIO build. Message parsers for the publish-subscribe framework
will be automatically generated in `build/generated`. `build/generated/include`
is specified as an include path in the Makefile, so `#include` directives can
start from that directory.

## Test

* `./gradlew-test.sh`

This runs a desktop build and executes all the unit tests in `src/test`.

## Deploy

* `./gradlew deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at
10.35.12.2, and restarts it.

## Documentation

* `./gradlew-docs.sh`

Doxygen 1.8.18 needs to be installed. The HTML documentation will be generated
in `docs/html` with an index.html page as the root.

## Game

The game for 2020 is called Infinite Recharge, where teams are tasked with shooting power cells into a low, high outer, and high inner goal. This year, the autonomous period returns and is the standard 15 seconds. Teams earn points in this period from moving off the initiation line and earn double the usual points for scoring in the power cell goals. Tele-op adds access to the control panel, which can be spun 3 to 5 times once the stage 2 capacity has been reached for more points, and can spun to a specified color from the FMS to score more points and earn the alliance a ranking point. Endgame tasks robots to climb a "generator switch" square truss, which may swing to be balanced or unbalanced. A ranking point is given if three robots are able to climb, or if two robots climb and the shield generator is balanced.

## Unique features

This years robot's unique features include:

- Augmented state-space drivetrain controller
- Vectored intake/outtake wheels
- Funnel
- Indexing conveyor
- Turret
- Single-wheel flywheel (similar to 2012)
- Two-stage elevator climbing system
- Raspberry Pi 3 w/ vision processing

## Goals of the year

|Status|Goal|Additional Description|
|------|----|----------------------|
|Yes|Drivetrain State-Space Controller|Following set autonomous trajectories accurately and precisely and using a nonlinear observer for a global pose estimate.|
|Yes|Turret State-Space Controller|Autoaiming at at the goal while the drivetrain moves underneath.|
|Yes|Flywheel State-Space Controller|Maintaining a constant RPM with reasonable recovery time.|
|Yes|Computer Vision|Using Perspective-n-Point to correct nonlinear observer's state estimate.|
|Yes|Unit Tests|Simulations for *everything* to test complicated robot code with no physical robot present.|

## Roster

Mentors: Tyler Veness

Students: William Jin (Lead), Kyle Quinlan, Matthew Santana, Ivy Quach, Adan Silva
