#!/usr/bin/env python3

import subprocess

import frcutils

subprocess.run([
    "ssh", f"lvuser@{frcutils.get_roborio_ip()}", "tail", "-f",
    "FRC_UserProgram.log"
])
