#!/usr/bin/env python3
"""Grabs latest build version string from the wpilib maven repo, finds the
corresponding commit in the allwpilib Git repo, then prints the version string,
commit hash, and a short description.

Release versions are of the form "[0-9]{4}\.[0-9]+\.[0-9]+". Development
versions are of the form "[0-9]{4}\.[0-9]+\.[0-9]+-[0-9]+-g[a-f0-9]+".

To use a release version in a robot program, add the following to build.gradle:
```
wpi.wpilibVersion = 'version string'
wpi.wpimathVersion = 'version string'
```

To use a specific development version in a robot program, add the following to
build.gradle:
```
wpi.wpilibVersion = 'version string'
wpi.wpimathVersion = 'version string'
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
        "https://frcmaven.wpi.edu:443/artifactory/development/edu/wpi/first/wpilibc/wpilibc-cpp/maven-metadata.xml"
    ).text

    # Get the first <latest> tag in the XML, which will be the latest version
    version = re.search(r"<latest>([^<]+)</latest>", content).group(1)
    print(version)

    # Get longer commit hash and short description from Git
    devel_version_match = re.search(r"-g([a-f0-9]{7})$", version)
    if devel_version_match:
        commit = devel_version_match.group(1)
    else:
        commit = f"v{version}"
    msgs = subprocess.check_output(
        ["git", "log", "--pretty=oneline", "-1",
         commit]).decode().rstrip().split(maxsplit=1)
    print(f"\tcommit {msgs[0]}")
    print(f"\t{msgs[1]}")


if __name__ == "__main__":
    main()
