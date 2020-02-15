#!/usr/bin/env python3

import argparse
import multiprocessing as mp
import os
from pathlib import Path
import re
import subprocess
import sys
from textwrap import TextWrapper
from urllib.request import urlretrieve


def purge(dir, pattern):
    for f in os.listdir(dir):
        if re.search(pattern, f):
            os.remove(os.path.join(dir, f))


def scrub_make_dep_rules():
    """Remove GNU Make dependencies that no longer exist from .d files"""
    # Get list of files in current directory
    files = [os.path.join(dp, f) for dp, dn, fn in os.walk(".") for f in fn]

    # Filter list to .d files
    file_rgx = re.compile(r"\.d$")
    files = [f for f in files if file_rgx.search(f)]

    for filename in files:
        with open(filename) as f:
            input = f.read()

        target = input[:input.find(":")]
        dep_string = input[input.find(":") + 1:]

        # Delete the .d file if its target no longer exists
        if not os.path.exists(target):
            os.remove(filename)
            continue

        # Extract a list of dependencies. Filenames can be delimited by a '\',
        # '\n', or ' '.
        deps = list(filter(None, re.split(r"\\|\n| ", dep_string)))

        # Remove nonexistent files from dependency list
        deps = [dep for dep in deps if os.path.exists(dep)]

        # Reconstruct .d file. Add target to beginning of list so it gets
        # wrapped.
        deps.insert(0, target + ":")
        wrapper = TextWrapper(break_long_words=False, break_on_hyphens=False, width=79 - len(" \\"))
        output = " \\\n ".join(wrapper.wrap(" ".join(deps))) + "\n"

        # If .d file changed, rewrite it
        if input != output:
            with open(filename, "w") as f:
                f.write(output)


def dl_progress(count, block_size, total_size):
    percent = min(int(count * block_size * 100 / total_size), 100)
    sys.stdout.write(f"\r-> {percent}%")
    sys.stdout.flush()


def download_file(maven_url, filename, dest_dir):
    """Download file from maven server.

    Keyword arguments:
    filename -- file to download
    dest_dir -- destination directory for file
    """
    dest_file = f"{dest_dir}/{filename}"
    if not os.path.exists(f"{dest_dir}/{filename}"):
        print(f"Downloading {filename}...")
        print(f"Querying {maven_url}/{filename}")
        urlretrieve(
            url=f"{maven_url}/{filename}", filename=dest_file, reporthook=dl_progress
        )
        print(" done.")

    folder_name = f"{dest_dir}/{os.path.splitext(filename)[0]}"
    if not os.path.exists(folder_name):
        print(f"Unzipping {dest_file}...", end="")
        sys.stdout.flush()
        subprocess.run(["unzip", "-q", "-d", folder_name, f"{dest_file}"])
        print(" done.")


def download_lib(maven_url, artifact_name, version, classifier, headers=True):
    maven_url += f"/{artifact_name}/{version}"

    # Download libs
    filename = f"{artifact_name}-{version}-{classifier}.zip"
    zip_name = f"build/{filename}"
    download_file(maven_url, filename, "build")

    # Download headers
    if headers:
        filename = f"{artifact_name}-{version}-headers.zip"
        zip_name = f"build/{filename}"
        download_file(maven_url, filename, "build")


def main():
    parser = argparse.ArgumentParser(description="Builds and deploys FRC C++ programs")
    parser.add_argument(
        "target",
        choices=["build", "deploy", "clean", "ci", "test"],
        help="""'build' compiles the robot program for athena and downloads missing dependencies.
        'deploy' compiles the program if it hasn't already and deploys it to a roboRIO.
        'clean' removes all build artifacts from the build folder.
        'ci' compiles the robot program for x86-64 and downloads missing dependencies.
        'test' compiles the robot program for x86-64 and downloads missing dependencies, then runs the tests.""",
    )
    parser.add_argument(
        "-j",
        dest="jobs",
        type=int,
        default=mp.cpu_count(),
        help="number of jobs to run (default is number of cores)")
    args = parser.parse_args()

    if not os.path.exists("build/"):
        os.makedirs("build/")

    WPI_MAVEN_URL = "https://frcmaven.wpi.edu/artifactory/release"
    REV_MAVEN_URL = "http://www.revrobotics.com/content/sw/max/sdk/maven"
    WPI_URL = WPI_MAVEN_URL + "/edu/wpi/first"
    OPENCV_URL = WPI_MAVEN_URL + "/edu/wpi/first/thirdparty/frc2020"
    GTEST_URL = WPI_MAVEN_URL + "/edu/wpi/first/thirdparty/frc2020"
    REV_URL = REV_MAVEN_URL + "/com/revrobotics/frc"

    WPI_VERSION = "2020.2.2"

    if args.target in ["build", "deploy"]:
        classifier = "linuxathena"
    else:
        classifier = "linuxx86-64"

    download_lib(WPI_URL + "/wpilibc", "wpilibc-cpp", WPI_VERSION, classifier)
    download_lib(WPI_URL + "/cameraserver", "cameraserver-cpp", WPI_VERSION, classifier)
    download_lib(WPI_URL + "/ntcore", "ntcore-cpp", WPI_VERSION, classifier)
    download_lib(WPI_URL + "/hal", "hal-cpp", WPI_VERSION, classifier)
    download_lib(WPI_URL + "/cscore", "cscore-cpp", WPI_VERSION, classifier)
    download_lib(WPI_URL + "/wpiutil", "wpiutil-cpp", WPI_VERSION, classifier)
    download_lib(OPENCV_URL + "/opencv", "opencv-cpp", "3.4.7-2", classifier)

    if args.target == "build":
        download_lib(WPI_URL + "/ni-libraries", "chipobject", "2020.9.2", classifier)
        download_lib(WPI_URL + "/ni-libraries", "netcomm", "2020.9.2", classifier)
        download_lib(
            WPI_URL + "/ni-libraries", "runtime", "2020.10.1", classifier, False
        )
        download_lib(WPI_URL + "/ni-libraries", "visa", "2020.10.1", classifier)
    elif args.target in ["ci", "test"]:
        download_lib(GTEST_URL, "googletest", "1.9.0-4-437e100-1", classifier + "static")

    classifier += "static"
    download_lib(REV_URL, "SparkMax-cpp", "1.5.1", classifier)
    download_lib(REV_URL, "SparkMax-driver", "1.5.1", classifier)

    # Generate pubsub messages
    if (
        not os.path.exists("build/generated")
        or os.path.getmtime("msgs") > os.path.getmtime("build/generated")
        or os.path.getmtime("python/generate_messages.py")
        > os.path.getmtime("build/generated")
    ):
        print("Generating PubSub messages...", end="")
        subprocess.run(
            [
                sys.executable,
                "python/generate_messages.py",
                "--input",
                "msgs",
                "--output",
                "build/generated",
            ]
        )
        subprocess.run(["touch", "build/generated"])
        print(" done.")

    make_athena = ["make", "-f", "mk/Makefile-linuxathena"]
    make_x86_64 = ["make", "-f", "mk/Makefile-linuxx86-64"]

    scrub_make_dep_rules()

    if args.target == "build":
        subprocess.run(make_athena + ["build", f"-j{args.jobs}"])
    elif args.target == "deploy":
        subprocess.run(make_athena + ["deploy", f"-j{args.jobs}"])
    elif args.target == "clean":
        subprocess.run(make_athena + ["clean"])
        subprocess.run(make_x86_64 + ["clean"])
    elif args.target == "ci":
        subprocess.run(make_x86_64 + ["build", f"-j{args.jobs}"], check=True)
    elif args.target == "test":
        subprocess.run(make_x86_64 + ["build", f"-j{args.jobs}"], check=True)

        # Remove old log files
        purge(".", r"\.csv")
        purge(".", r"Robot\.log$")

        subprocess.run(["build/linuxx86-64/frcUserProgram"], check=True)


if __name__ == "__main__":
    main()
