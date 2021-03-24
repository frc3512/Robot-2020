#!/usr/bin/env python3

import subprocess

import frcutils

ip = frcutils.get_roborio_ip()

# Stop lvrt-daemon and lvrt
subprocess.run(["ssh", f"admin@{ip}", "killall", "lvrt-daemon"])
subprocess.run(["ssh", f"admin@{ip}", "killall", "lvrt"])

# Delete new lvrt
subprocess.check_output(["ssh", f"admin@{ip}", "rm", "-f", "/home/lvuser/lvrt"])

# Replace "exec /home/lvuser/lvrt" in /etc/init.d/lvrt-wrapper with original
subprocess.run([
    "ssh", f"admin@{ip}", "sed", "-i",
    "'s!exec /home/lvuser/lvrt!exec ./lvrt!'", "/etc/init.d/lvrt-wrapper"
])

# Restart lvrt-daemon
subprocess.check_output(["ssh", f"admin@{ip}", "reboot"])
