# System architecture

This document describes the robot's overall system architecture.

- [System architecture](#system-architecture)
  * [TimesliceRobot class](#timeslicerobot-class)
  * [Subsystem overview](#subsystem-overview)
    + [Vision](#vision)
    + [Drivetrain](#drivetrain)
    + [Flywheel](#flywheel)
    + [Turret](#turret)
    + [Intake](#intake)
    + [Climber](#climber)
  * [Subsystem controllers](#subsystem-controllers)
    + [Targeting information flow](#targeting-information-flow)
    + [Flywheel controller implementation](#flywheel-controller-implementation)
    + [Turret controller implementation](#turret-controller-implementation)
    + [Drivetrain controller implementation](#drivetrain-controller-implementation)
  * [Background on real-time scheduling](#background-on-real-time-scheduling)
  * [Real-time configuration](#real-time-configuration)
    + [Main robot thread](#main-robot-thread)
    + [Logging](#logging)
    + [Automomous](#automomous)
    + [ADIS16470 IMU thread](#adis16470-imu-thread)
    + [FRC_NetCommDaemon process](#frc-netcommdaemon-process)

## TimesliceRobot class

The main robot class inherits from
[TimesliceRobot](../src/main/include/frc/TimesliceRobot.h), so it supports the
following modes:

* Disabled (the robot doesn't move)
* Autonomous (the robot follows preprogrammed instructions)
* Teleop (the robot follows driver commands)
* Test (the robot executes diagnostic code)

The following overridable member functions are called on mode entry:

* DisabledInit()
* AutonomousInit()
* TelopInit()
* TestInit()

The following overridable member functions are called every 20 ms while in that
mode:

* DisabledPeriodic()
* AutonomousPeriodic()
* TeleopPeriodic()
* TestPeriodic()

## Subsystem overview

This robot has six subsystems: drivetrain, flywheel, turret, intake, and
climber, which all inherit from
[SubsystemBase](../src/main/include/subsystems/SubsystemBase.hpp) to provide
"init" and "periodic" functions for each mode just like those in TimesliceRobot.

Functions that access multiple subsystems at once like the shooting state
machine or autonomous modes exist in the Robot class instead of a subsystem.

### Vision

We run [PhotonVision](https://docs.photonvision.org/en/latest/) on a
[Gloworm](https://gloworm.vision/) to obtain 3D transformations from the
camera's coordinate frame to the target's coordinate frame (see solvePnP() in
the [OpenCV docs](https://docs.opencv.org/4.5.3/d9/d0c/group__calib3d.html)).

Every 20 ms, the Vision subsystem receives the camera-to-target transformations
via NetworkTables, converts them to drivetrain pose measurements in the global
coordinate frame, then pushes them into a queue for the Drivetrain subsystem to
consume.

### Drivetrain

The Drivetrain subsystem handles teleop driving via joysticks and autonomous
driving along a desired path specified by Hermite splines.

It can take either clamped cubic Hermite splines (poses at the endpoints and
translations at the interior waypoints), or quintic Hermite splines (poses at
both the endpoints and interior waypoints). When the user doesn't care about the
heading at interior waypoints, cubic splines should be prefered because the
interior heading will be automatically chosen such that the change in curvature,
and thus the change in drivetrain wheel speed, is minimized between the
waypoints.

### Flywheel

The Flywheel subsystem has a flywheel which is spun up to a desired RPM to
launch foam balls at a target.

### Turret

The Turret subsystem aims the flywheel.

### Intake

The Intake subsystem contains the logic controlling the intake rollers, funnel,
and conveyor belt tower. IR beam sensors on the top and bottom of the conveyor
belt tower are used to automatically index balls after they pass the intake
rollers. See [this I/O table](intake-io-table.md) for the logic we implemented.

### Climber

The Climber subsystem controls an elevator for hanging and balancing on a
tilting bar (like an inverted teeter-totter). The elevator assembly has a wheel
protruding from the front that's used for traversing the bar and spinning the
control panel to a color determined by the Field Management System (FMS).

## Subsystem controllers

The subsystems that have state-space controllers (drivetrain, flywheel, and
turret) inherit from
[ControlledSubsystemBase](../src/main/include/subsystems/ControlledSubsystemBase.hpp),
which provides logging for the references, states, inputs, and outputs of the
controller. CSV and NetworkTables endpoints are supported.

Each controlled subsystem's implementation of
`ControlledSubsystemBase::ControllerPeriodic()` is given a timeslot within which
to run every 5 ms while the robot is enabled. The timeslot ordering and duration
are documented in the timeslot allocation table in the Robot constructor. The
timeslots are scheduled using `TimesliceRobot::Schedule()`.

Each controlled subsystem has a corresponding controller class (e.g., Flywheel
has FlywheelController). Each controller class inherits from
[ControllerBase](../src/main/include/controllers/ControllerBase.hpp), which
requires a Calculate() function and contains some bookkeeping variables for
controllers.

The subsystem controllers are run in the following order:

* DrivetrainControler
* FlywheelController
* TurretController

Each subsystem's `ControllerPeriodic()` performs a state observer prediction
based on the previous run's motor outputs, performs a state observer update
based on the current run's measurements, calls the controller's Calculate()
function, then sets the motor outputs.

### Targeting information flow

The turret uses the drivetrain position relative to the target and the flywheel
speed to determine where to aim. Therefore:

1. The Vision subsystem retrieves new camera-to-target transformations from
   PhotonVision, converts them to drivetrain global pose measurements, then
   pushes them into a queue for the Drivetrain subsystem to consume.
2. The drivetrain pose is updated using encoder measurements, IMU measurements,
   and any existing vision data in the queue.
3. The flywheel speed is updated based on a lookup table (LUT) of
   range-to-target to flywheel speed mappings.
4. The turret is aimed using the drivetrain's pose estimate. A correction factor
   incorporating the flywheel's speed is applied if the drivetrain is moving
   relative to the target (see [this derivation](turret-aim-while-moving.md)).

### Flywheel controller implementation

The flywheel uses a Kalman filter for state estimation, an LQR for feedback
control, and a plant inversion feedforward to maintain steady-state velocity.

### Turret controller implementation

The turret uses a Kalman filter for state estimation and an LQR for feedback
control. When the drivetrain pose isn't being used for aiming (which provides an
implicit motion profile), a trapezoidal motion profile is applied to references
so a plant inversion feedforward can be effectively applied at all times.

The heading and angular rate references take into account the drivetrain's
current velocity so it won't overshoot as the drivetrain moves.

### Drivetrain controller implementation

The drivetrain uses an unscented Kalman filter for state estimation and a linear
time-varying LQR for feedback control. Since the model is control-affine (the
dynamics are nonlinear, but the control inputs provide a linear contribution), a
plant inversion feedforward was used. Trajectories generated from splines
provide the motion profile to follow.

The linear time-varying controller has a similar form to the LQR, but the model
used to compute the controller gain is the nonlinear model linearized around the
drivetrain's current state. We precomputed gains for important places in our
state-space, then interpolated between them with a LUT to save computational
resources.

We decided to control for longitudinal error and cross-track error in the
chassis frame instead of x and y error in the global frame, so the state
Jacobian simplified such that we only had to sweep velocities (-4m/s to 4m/s).

See section 9.6 in [Controls Engineering in
FRC](https://file.tavsys.net/control/controls-engineering-in-frc.pdf) for a
derivation of the control law we used shown in theorem 9.6.3.

## Background on real-time scheduling

Our robot code runs with
[real-time priority](https://frc3512.github.io/ci/intro-to-real-time-software/).
We use the first in-first out (FIFO) scheduler.

The following docs are paraphrased from `man 7 sched`.

SCHED_FIFO can be used only with static priorities higher than 0 (1 to 99 with
99 being highest), which means that when a SCHED_FIFO thread becomes runnable,
it will always immediately preempt any currently running non-real-time thread.
For threads scheduled under the SCHED_FIFO policy, the following rules apply:

1) A running SCHED_FIFO thread that has been preempted by another thread of
   higher priority will stay at the head of the list for its priority and will
   resume execution as soon as all threads of higher priority are blocked again.
2) When a blocked SCHED_FIFO thread becomes runnable, it will be inserted at the
   end of the list for its priority.
3) If the priority of the running or runnable SCHED_FIFO thread is changed, the
   effect on the thread's position in the list depends on the direction of the
   change to the thread's priority:
   * If the thread's priority is raised, it is placed at the end of the list for
     its new priority. As a consequence, it may preempt a currently running
     thread with the same priority.
   * If the thread's priority is unchanged, its position in the run list is
     unchanged.
   * If the thread's priority is lowered, it is placed at the front of the list
     for its new priority.
4) A thread that yields by calling `std::this_thread::yield()` will be put at
   the end of the list.

No other events will move a thread scheduled under the SCHED_FIFO policy in the
wait list of runnable threads with equal static priority.

A SCHED_FIFO thread runs until either it is blocked by an I/O request, it is
preempted by a higher priority thread, or it yields.

See `man 7 sched` for other scheduler types.

## Real-time configuration

See [RealTimePriorities.hpp](../src/main/include/RealTimePriorities.hpp) for the
full list of thread and process priorities.

### Main robot thread

This is the main thread of TimesliceRobot on which the subsystems and
controllers run. This thread is given a priority of 15.

This thread runs on what is called a Notifier. The WPILib HAL Notifier API
maintains a [priority queue](https://en.wikipedia.org/wiki/Priority_queue) of
user Notifier threads sorted by wakeup time, and the FPGA timer interrupt is
configured to wake the HAL Notifier thread up when it's the soonest user
thread's time to run. When a Notifier is woken up, the periodic wakeup duration
is added to the previous wakeup time to produce the next wakeup time, then it's
reinserted into the priority queue.

To ensure real-time threads like the main robot thread are woken up in a timely
manner, the HAL Notifier thread is given an RT priority of 40 (i.e., the HAL
Notifier thread should always preempt user Notifier threads to wakeup others
that may have a higher priority).

### Logging

To avoid affecting the scheduling and runtime of the controllers with file and
network I/O, the CSV and NetworkTables logging is performed on a separate thread
at a lower priority than the main robot thread (14).

### Automomous

Our autonomous runs in a separate thread, but the main robot thread and the
autonomous thread yield to each other when they need to run. They are given
equal priorities to facilitate this, and given the coroutine abstraction we have
over threads in [AutonomousChooser](../src/main/include/AutonomousChooser.hpp),
no explicit thread synchronization is needed in user code.

When the robot code starts autonomous, it awaits on the autonomous function.
That function runs until it needs to wait for an action to complete (like the
drivetrain driving to a point or the flywheel spinning up), then it yields to
the main robot thread so it can update controllers and do other work. During the
next iteration of the main robot loop, the autonomous mode is resumed where it
left off by awaiting on the function again. This style of programming is known
as a [coroutine](https://en.wikipedia.org/wiki/Coroutine) (which we mimic using
an abstraction over threads).

### ADIS16470 IMU thread

The ADIS16470 IMU thread is given a priority of 33 so SPI gyro measurements are
incorporated before any other user code tries to sample the gyro, but the HAL
Notifier thread can still preempt to wake up user threads.

### FRC_NetCommDaemon process

The FRC_NetCommDaemon process is given a priority of 35 so it's not preempted by
user code during high CPU utilization.
