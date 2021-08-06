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
* scp (optional, for CSV logging)

Install the following python packages via `pip3 install --user package_name`.

* wpiformat (optional)
  * https://github.com/wpilibsuite/styleguide/blob/main/wpiformat/README.rst

We use the latest version of clang-format with wpiformat for C++ formatting.

## C++ intellisense in Vim with clangd

VSCode's intellisense is occasionally unreliable. Vim + clangd is an alternative
that provides better intellisense because it actually invokes a C++ compiler
frontend. It also generates linting annotations via clang-tidy. Setup is as
follows.

1. Install Node.js. You should have a `node` executable in PATH.
2. Install clangd, which comes with the
   [LLVM compiler](https://releases.llvm.org/download.html)
3. Add `Plugin 'neoclide/coc.nvim'` to your `.vimrc`
4. In vim, run `:CocInstall coc-clangd`
5. Run `./gradlew intellisense`

The clangd indexer will start when a file is opened in Vim. See
https://github.com/clangd/coc-clangd for troubleshooting steps if needed.

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

### roboRIO imaging

By default, our deploy process overwrites robotCommand on the roboRIO to disable
the NI web server. This is required for imaging, so to reenable it, run

* `./gradlew deploy -PallowImaging`

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

### Debugger

* `./buildscripts/debug_test.py`

This runs a debug build of the tests in GDB. Once the build completes and the
debugger's prompt appears, enter `run` to start the robot program. It may take a
while due to the debugger having to load a lot of symbols. If the robot code
crashes, enter `bt` to get a backtrace.

* `./buildscripts/debug_simulation.py`

This runs a debug desktop build of the robot code and simulation GUI in a
debugger.

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

## Live logging

Logging can be viewed while the robot is running and in simulation.
**OutlineViewer**, **CSV**, and **Glass** are the main ways to view logs while
the robot is running. ControllerBase supports two logging backends for
high-throughput controller performance data: **CSV** and **Glass**.

### OutlineViewer

OutlineViewer is a WPILib tool to view NetworkTables.

* Make sure to have FRC toolchain from the setup section.
* Open the tools directory `~/wpilib/2020/tools` and run
  `python3 ToolsUpdater.py`.
* Open OutlineViewer by running `python3 OutlineViewer.py` and set the server
  location to 10.35.12.2. The default port will work.

### CSV

This backend writes CSV files to the roboRIO flash storage. After they are
recorded, they can be retrieved with `tools/get_csvs.py` and displayed with
`tools/plot_subsystems.py`.

### Glass

Glass is a WPILib tool that allows for pose visualization and networktable
plotting/visualization while the robot is running. See the
[Glass documentation](https://docs.wpilib.org/en/latest/docs/software/dashboards/glass/introduction.html)
for more details.

* Make sure to have FRC toolchain from the setup section.
* Open the tools directory `~/wpilib/2020/tools` and run
  `python3 ToolsUpdater.py`.
* Open OutlineViewer by running `python3 Glass.py` and set the server
  location to 10.35.12.2. The default port will work.

## Simulation logging

Logs can be viewed in real time via NetworkTables in the simulation GUI or
offline via CSV processing.

### Simulation GUI

The simulation GUI is straightforward but can be read more about
[here](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/simulation-gui.html).

### CSV

After running the tests, the CSV files will be saved. The backend writes CSV
files to `build/test-results/frcUserProgramTest`. To display the CSVs, run the
following command:

```
./tools/plot_subsystems.py [regexp]
```

`plot_subsystems.py` will display CSVs whose filepaths match the optional
regular expression `[regexp]`. It should be given filepath components after the
`frcUserProgramTest` folder.

To show data for a specific subsystem, include its name in the regular
expression.

```
./tools/plot_subsystems.py Flywheel
```

Specific states, inputs, or outputs can be viewed as well.

```
./tools/plot_subsystems.py "Flywheel states"
```

Other examples:

```
./tools/plot_subsystems.py "DrivetrainTest/ReachesReferenceStraight"
./tools/plot_subsystems.py "AutonomousTests/AutonomousTest/Run/Left Side Shoot Ten/Drivetrain (States|Outputs)"
./tools/plot_subsystems.py "DrivetrainTest/ReachesReferenceCurve/Drivetrain timing"
```

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
