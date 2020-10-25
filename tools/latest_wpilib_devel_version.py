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

    # Get list of one-line commit messages sorted from earliest to latest
    tag = "v2020.3.2"
    commits = (subprocess.check_output(
        ["git", "log", "-q", "--reverse", "--pretty=oneline",
         f"{tag}..HEAD"]).decode().strip().split("\n"))

    content = requests.get(
        "https://frcmaven.wpi.edu:443/artifactory/development/edu/wpi/first/wpimath/wpimath-cpp/maven-metadata.xml"
    ).text

    # Get the first <version> tag in the XML, which will be the latest
    # version
    version_rgx = re.compile(
        r"""<version>
        (?P<version>[0-9\.]+-(?P<commit>[0-9]+)-g[a-f0-9]+)
        </version>""",
        re.VERBOSE,
    )
    match = version_rgx.search(content)
    version = match.group("version")
    commit = int(match.group("commit"))

    print(f"{version}")
    match = re.search(r"^([a-f0-9]+)\s+(.*)$", commits[commit - 1])
    print(f"\tcommit {match.group(1)}")
    print(f"\t{match.group(2)}")


if __name__ == "__main__":
    main()
