#!/usr/bin/env python3
"""Grabs latest development build version string from the wpilib maven repo,
finds the corresponding commit in the allwpilib Git repo, then prints the
version string, commit hash, and a short description.

To use it in a robot program, add the following to build.gradle:
```
wpi.wpilibVersion = 'version string'
wpi.maven.useDevelopment = true
```
"""

import os
import re
import requests
import subprocess
import tempfile


def clone_repo(url, branch):
    repo = os.path.basename(url)

    # Clone Git repository into current directory or update it
    if not os.path.exists(repo):
        dest = os.path.join(os.getcwd(), repo)
        subprocess.run(["git", "clone", url, dest])
        os.chdir(repo)
    else:
        os.chdir(repo)
        subprocess.run(["git", "pull", "-q"])


def main():
    os.chdir(tempfile.gettempdir())
    clone_repo("git://github.com/wpilibsuite/allwpilib", "master")

    content = requests.get(
        "https://frcmaven.wpi.edu:443/artifactory/development/edu/wpi/first/wpimath/wpimath-cpp/maven-metadata.xml"
    ).text

    # Get the first <version> tag in the XML, which will be the latest version
    version_rgx = re.compile(
        r"""(?<=<version>)[0-9\.]+-[0-9]+-g([a-f0-9]+)(?=</version>)""",
        re.VERBOSE,
    )
    match = version_rgx.search(content)
    print(f"{match.group()}")

    # Get longer commit hash and short description from Git
    log_msg = (subprocess.check_output(
        ["git", "log", "--pretty=oneline", "-1",
         match.group(1)]).decode().strip())
    msgs = log_msg.split(maxsplit=1)
    print(f"\tcommit {msgs[0]}")
    print(f"\t{msgs[1]}")


if __name__ == "__main__":
    main()
