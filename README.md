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
* rsync (optional, for CSV logging)

Install the following python packages via `pip3 install --user package_name`.

* wpiformat (optional)
  * https://github.com/wpilibsuite/styleguide/blob/master/wpiformat/README.rst

We use the latest version of clang-format with wpiformat for C++ formatting.

## Build options

### Build everything

* `./gradlew build`

This runs a roboRIO and desktop build and runs the desktop tests. This may take
a while, so more specific builds (see below) are recommended instead.

### Build (athena)

* `./gradlew-build-athena.sh`

This runs a roboRIO build.

### Deploy

* `./gradlew deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at
10.35.12.2, and restarts it.

### Test

* `./gradlew-test.sh`

This builds the robot code's unit tests in `src/test` and runs them. They are
useful for ensuring parts of the robot code continue to work correctly after
implementing new features or refactoring existing functionality.

### Simulation GUI

* `./gradlew-simgui.sh`

This builds the robot code for desktop and runs the simulation GUI.

### Documentation

* `./gradlew-docs.sh`

Doxygen 1.8.20 needs to be installed. This generates HTML documentation for the
robot code from in-source Doxygen comments and places it in `docs/html` with an
index.html page as the root.

### GDB

* `./gradlew-gdb.sh`

This runs a debug build of the robot code in the GNU debugger (GDB). Once the
build completes and GDB's prompt appears, enter `run` to start the robot
program. It may take a while due to the debugger having to load a lot of
symbols. If the robot code crashes, enter `bt` to get a backtrace.

### Valgrind

`./gradlew-valgrind.sh`

Valgrind is useful for finding memory leaks, memory corruption, and reads from
uninitialized memory so they can be fixed.

## Logging

* Make sure to have FRC toolchain from the setup section.
* Open the tools directory `~/wpilib/2020/tools` and run
  `python3 ToolsUpdater.py`.
* Open OutlineViewer by running `python3 OutlineViewer.py` and set the server
  location to 10.35.12.2. The default port will work.

ControllerBase supports two logging backends for high-throughput controller
performance data: CSV and LiveGrapher. The active backend can be selected via
the `NETWORK_LOGGING` #define in
[ControllerBase.hpp](src/main/include/controllers/ControllerBase.hpp).

### CSV

This backend writes CSV files to the roboRIO flash storage. After they are
recorded, they can be retrieved with `tools/get_csvs.py` and displayed with
`tools/plot_subsystems.py`.

### LiveGrapher

This backend sends information in real-time to LiveGrapher clients connected to
the robot network. See the
[LiveGrapher README](https://github.com/frc3512/LiveGrapher#livegrapher) for
more.

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
