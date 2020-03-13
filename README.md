# FRC team 3512's 2020 robot

Source code for the 2020 comp robot: Andromeda

Source code also for the 2020 practice robot: Aithir

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

## Build

* `./make.py build`

This runs a roboRIO build. Message parsers for the publish-subscribe framework
will be automatically generated in `build/generated`. `build/generated/include`
is specified as an include path in the Makefile, so `#include` directives can
start from that directory.

## Test

* `./make.py test`

This runs a desktop build and executes all the unit tests in `src/test`.

## Deploy

* `./make.py deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at
10.35.12.2, and restarts it.

## Documentation

* `./make.py docs`

Doxygen 1.8.17 needs to be installed. The HTML documentation will be generated
in `docs/html` with an index.html page as the root.

## Game



## Unique features



## Goals of the year



## Roster

Mentors: Tyler Veness

Students:William Jin (Lead), Kyle Quinlan, Matthew Santana, Ivy Quach, Adan Silva
