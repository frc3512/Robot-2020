#!/bin/bash
HOST='10.35.12.2'

if [ $# -ne 1 ] ; then
    echo usage: ./deploy.sh DIRECTORY
    echo -e "DIRECTORY is the directory in which the robot code binary is located"
    exit 1
fi

ssh lvuser@$HOST "rm -f /home/lvuser/frcUserProgram"
ssh lvuser@$HOST "killall -q netconsole-host || :"
scp $1/frcUserProgram lvuser@$HOST:/home/lvuser
ssh admin@$HOST "setcap 'cap_sys_nice=pe' /home/lvuser/frcUserProgram"
ssh lvuser@$HOST ". /etc/profile.d/natinst-path.sh; chmod a+x /home/lvuser/frcUserProgram; /usr/local/frc/bin/frcKillRobot.sh -t -r;"
