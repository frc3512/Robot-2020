#!/usr/bin/env python3

import os
import subprocess


def main():
    # Stops crond and systemWebServer before starting robot program
    with open("robotCommand", "w") as output:
        output.write(""" \"/etc/init.d/crond stop\"
 \"/usr/local/natinst/etc/init.d/systemWebServer stop\"
 \"/home/lvuser/frcUserProgram\"
""")
    subprocess.check_output(
        ["scp", "robotCommand", "lvuser@10.35.12.2:/home/lvuser"])
    os.remove("robotCommand")


if __name__ == "__main__":
    main()
