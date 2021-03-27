#!/usr/bin/env python3
"""Retrieves all CSV log files from target."""

import os
import subprocess

import frcutils

if not os.path.exists("csvs"):
    os.makedirs("csvs")
subprocess.run(
    ["scp", f"lvuser@{frcutils.get_roborio_ip()}:/media/sda/*.csv", "csvs/"])
