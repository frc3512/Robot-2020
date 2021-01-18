# FRC team 3512's 2020 robot

Source code for the 2020 comp robot: Arcadia

Source code also for the 2020 practice robot: Ellinika

## Setup

Install the relevant FRC toolchain for your platform (see
https://github.com/wpilibsuite/allwpilib/releases). Make sure the toolchain is
placed in `~/wpilib/2020/roborio` (Linux) or
`C:\Users\Public\wpilib\2020\roborio` (Windows).

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

* `./gradlew buildAthena`

This runs a roboRIO build.

### Deploy

* `./gradlew deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at
10.35.12.2, and restarts it.

### Test

Unit tests are useful for ensuring parts of the robot code continue to work
correctly after implementing new features or refactoring existing functionality.

* `./gradlew test`

This runs a release build of the robot code's unit tests from `src/test`. This
is a shorthand for the `testRelease` task.

* `./gradlew testDebug`

This runs a debug build of the robot code's unit tests from `src/test`.

* `./gradlew testRelease`

This runs a release build of the robot code's unit tests from `src/test`.

### Simulation GUI

* `./gradlew simulateCpp`

This runs a debug build of the robot code in the simulation GUI.

### Documentation

* `./gradlew doxygen`

The source code and algorithms documentation is located in the [docs](docs)
folder. This command generates HTML documentation for the robot code from
in-source Doxygen comments. The results are placed in a `docs/html` folder with
an `index.html` page as the root.

### GNU debugger (GDB)

* `./buildscripts/gdb-test.sh`

This runs a debug build of the tests in GDB. Once the build completes and GDB's
prompt appears, enter `run` to start the robot program. It may take a while due
to the debugger having to load a lot of symbols. If the robot code crashes,
enter `bt` to get a backtrace.

* `./buildscripts/gdb-simulate.sh`

This runs a debug desktop build of the robot code and simulation GUI in GDB.

* `./buildscripts/gdb-test-ci.sh`

This runs a debug build of the tests in GDB in noninteractive mode. It will
automatically run the program in the debugger and print a backtrace if it
crashes. This is useful for debugging crashes in GitHub Actions.

### Address sanitizer

`./gradlew test -Pasan`

This runs a release build of the tests with the address sanitizer enabled. The
address sanitizer is useful for finding memory corruption and reads from
uninitialized memory so they can be fixed.

### Valgrind

`./gradlew valgrind`

This runs a release build of the tests in Valgrind. Valgrind is useful for
finding memory leaks, memory corruption, and reads from uninitialized memory so
they can be fixed.

## Logging

* Make sure to have FRC toolchain from the setup section.
* Open the tools directory `~/wpilib/2020/tools` and run
  `python3 ToolsUpdater.py`.
* Open OutlineViewer by running `python3 OutlineViewer.py` and set the server
  location to 10.35.12.2. The default port will work.

ControllerBase supports two logging backends for high-throughput controller
performance data: CSV and LiveGrapher.

### CSV

This backend writes CSV files to the roboRIO flash storage. After they are
recorded, they can be retrieved with `tools/get_csvs.py` and displayed with
`tools/plot_subsystems.py`.

### LiveGrapher

This backend sends information in real-time to LiveGrapher clients connected to
the robot network. See the
[LiveGrapher README](https://github.com/frc3512/LiveGrapher#livegrapher) for
more.

## Autonomous mode selection

Open shuffleboard and select the desired autonomous mode from the dropdown menu.
When the indicator next to the menu turns from a red X to a green checkmark, the
robot has confirmed the selection.

See
[this](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html)
for details on how the robot side works.

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
