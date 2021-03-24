#!/usr/bin/env python3

import os
import platform
import subprocess

import frcutils

ip = frcutils.get_roborio_ip()

# Make path to cross compiler
if platform.system() == "Linux" or platform.system() == "Darwin":
    home = os.environ["HOME"]
elif platform.system() == "Windows":
    home = os.environ["PUBLIC"]
gcc = os.path.join(home,
                   "wpilib/2021/roborio/bin/arm-frc2021-linux-gnueabi-gcc")

# Build new lvrt
with open("lvrt.c", "w") as output:
    output.write("""#include <stdlib.h>
#include <unistd.h>

int main(void) {
    while (system("/bin/sh -c /usr/local/frc/bin/frcRunRobot.sh")) {
        usleep(500 * 1e3);
    }
    return 0;
}
""")
subprocess.check_output([gcc, "lvrt.c", "-o", "lvrt"])
print("Compiled new LVRT. Logging into target...")

# Stop lvrt-daemon and lvrt
subprocess.run(["ssh", f"admin@{ip}", "killall", "lvrt-daemon"])
subprocess.run(["ssh", f"admin@{ip}", "killall", "lvrt"])

# Deploy new lvrt
subprocess.check_output(["scp", "lvrt", f"lvuser@{ip}:/home/lvuser"])

# Delete lvrt source files that are no longer needed
for f in ["lvrt.c", "lvrt"]:
    os.remove(f)

# Replace "exec ./lvrt" in /etc/init.d/lvrt-wrapper with new executable
subprocess.check_output([
    "ssh", f"admin@{ip}", "sed", "-i",
    "'s!exec ./lvrt!exec /home/lvuser/lvrt!'", "/etc/init.d/lvrt-wrapper"
])

# Restart lvrt-daemon
subprocess.check_output(["ssh", f"admin@{ip}", "reboot"])
