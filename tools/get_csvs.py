#!/usr/bin/env python3
"""Retrieves all CSV log files from target."""

import os
import subprocess

import frcutils

if not os.path.exists("csvs"):
    os.makedirs("csvs")

ip = frcutils.get_roborio_ip()

# If storage is mounted in /media, retrieve log files from there
dirs = (subprocess.check_output(["ssh", f"lvuser@{ip}", "ls", "/media"],
                                encoding="utf-8").rstrip().split("\n"))
if len(dirs) > 0:
    subprocess.run(["scp", f"lvuser@{ip}:/media/{dirs[0]}/*.csv", "csvs/"])
else:
    subprocess.run(["scp", f"lvuser@{ip}:*.csv", "csvs/"])
