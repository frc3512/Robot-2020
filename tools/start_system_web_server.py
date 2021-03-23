#!/usr/bin/env python3

import subprocess

import frcutils

subprocess.run([
    "ssh", f"admin@{frcutils.get_roborio_ip()}",
    "/usr/local/natinst/etc/init.d/systemWebServer", "start"
])
