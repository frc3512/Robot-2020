#!/bin/bash
set -e

# Give RT capability
ssh admin@10.35.12.2 setcap cap_sys_nice+eip /home/lvuser/frcUserProgram

# Disable crond because it uses 50% CPU and there's no cronjobs to run
ssh admin@10.35.12.2 /etc/init.d/crond stop

# Disable NI web server because it leaks memory badly
ssh admin@10.35.12.2 /usr/local/natinst/etc/init.d/systemWebServer stop
