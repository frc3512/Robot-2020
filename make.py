#!/usr/bin/env python3

import argparse
import multiprocessing as mp
import os
from pathlib import Path
import re
import requests
import subprocess
import sys
from textwrap import TextWrapper
from urllib.request import urlretrieve
import shutil


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


def download_NIPackage(target, pkg):
    """Download a NI package onto the RoboRio using ssh.
    Keyword arguements:
    target -- target to install package onto
    pkg -- name of package
    """
    print(f"Installing NI package...", pkg)
    PKG_URL = "https://download.ni.com/ni-linux-rt/feeds/2019/arm/cortexa9-vfpv3/" + pkg
    subprocess.check_call(["wget", PKG_URL, "-O", pkg])
    try:
        subprocess.check_call([
            "external/ssh/usr/bin/scp", "-S", "external/ssh/usr/bin/ssh", pkg,
            target + ":/tmp/" + pkg
        ])
        subprocess.check_call([
            "external/ssh/usr/bin/ssh", target, "opkg", "install",
            "/tmp/" + pkg
        ])
        subprocess.check_call(
            ["external/ssh/usr/bin/ssh", target, "rm", "/tmp/" + pkg])
    finally:
        subprocess.check_call(["rm", pkg])

        
def download_file(maven_url, filename, dest_dir):
    """Download file from maven server.

    Keyword arguments:
    filename -- file to download
    dest_dir -- destination directory for file
    """
    dest_file = f"{dest_dir}/{filename}"
    if not os.path.exists(dest_file):
        print(f"[maven] {filename}")

        r = requests.get(f"{maven_url}/{filename}", stream=True)
        block_size = 8196
        valid_size = True
        try:
            total_size = int(r.headers["content-length"])
        except KeyError:
            valid_size = False

        status_line = "\r  downloading... "
        with open(dest_file, "wb") as f:
            for count, chunk in enumerate(r.iter_content(block_size)):
                if valid_size:
                    percent = min(int(count * block_size * 100 / total_size), 100)
                    print(status_line + f"{percent}%", end="")
                    sys.stdout.flush()
                f.write(chunk)

        status_line += "100%"
        print(status_line, end="")
        sys.stdout.flush()

    folder_name = f"{dest_dir}/{os.path.splitext(filename)[0]}"
    if not os.path.exists(folder_name):
        status_line += ", unzipping..."
        print(status_line, end="")
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
        choices=["build", "deploy", "clean", "ci", "test", "docs"],
        help="""'build' compiles the robot program for athena and downloads missing dependencies.
        'deploy' compiles the program if it hasn't already and deploys it to a roboRIO.
        'clean' removes all build artifacts from the build folder.
        'ci' compiles the robot program for x86-64 and downloads missing dependencies.
        'test' compiles the robot program for x86-64 and downloads missing dependencies, then runs the tests.
        'docs' generates C++ API documentation using Doxygen.""",
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
    
    if not os.path.exists("deploy/"):
        os.makedirs("deploy/")

    WPI_MAVEN_URL = "https://frcmaven.wpi.edu/artifactory/release"
    REV_MAVEN_URL = "http://www.revrobotics.com/content/sw/max/sdk/maven"
    WPI_URL = WPI_MAVEN_URL + "/edu/wpi/first"
    OPENCV_URL = WPI_MAVEN_URL + "/edu/wpi/first/thirdparty/frc2020"
    GTEST_URL = WPI_MAVEN_URL + "/edu/wpi/first/thirdparty/frc2020"
    REV_URL = REV_MAVEN_URL + "/com/revrobotics/frc"

    WPI_VERSION = "2020.3.2"
    ROBORIO_USER = "lvuser"
    ROBORIO_IP_ADDRESS = "10.35.12.2"
    ROBORIO_TARGET_DIR = "/home/lvuser"

    target_dir = ROBORIO_TARGET_DIR
    user = ROBORIO_USER
    ip_address = ROBORIO_IP_ADDRESS
    ssh_target = "%s@%s" % (user, ip_address)
    copy_dir = "%s:%s" % (ssh_target, target_dir)

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
        download_lib(GTEST_URL, "googletest", "1.9.0-4-437e100-1", classifier + "static")

    classifier += "static"
    download_lib(REV_URL, "SparkMax-cpp", "1.5.2", classifier)
    download_lib(REV_URL, "SparkMax-driver", "1.5.2", classifier)

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
        #Generates a list of files that are found in the deploy folder
        files = [os.path.join(dp, f) for dp, dn, fn in os.walk("deploy/") for f in fn]
        if len(files) > 0:
            print(f"Files found! Attempting to copy files")
            #Establish a connection to the roboRIO
            print(f"Establishing connection to the RoboRIO")
            subprocess.run(["ssh", ssh_target])
            print(f"Checking rsync is installed on the roboRIO...")
            #Command we used to test if rsync is on the roboRIO
            rsync_cmd = ("rsync", "--version")
            try:
                #Checks to see if the called command returns successfully
                subprocess.check_call(rsync_cmd)
                print(f"Command rsync found!")
            except subprocess.CalledProcessError as e:
                if e.returncode == 127:
                    #Downloads rsync + its dependencies if they are not present on the roboRIO
                    print("Command rsync not found! Installing necessary packages...")
                    download_NIPackage(ssh_target, "libattr1_2.4.47-r0.513_cortexa9-vfpv3.ipk")
                    download_NIPackage(ssh_target, "libacl1_2.2.52-r0.183_cortexa9-vfpv3.ipk")
                    download_NIPackage(ssh_target, "rsync_3.1.3-r0.6_cortexa9-vfpv3.ipk")
                    subprocess.check_call(rsync_cmd)
                else:
                    raise e;
            #Copy of all the files in the deploy folder onto the roboRIO
            subprocess.run(["rsync", "-vazh", "ssh", "--progress", target_dir, copy_dir])
        else:
            #If no files are found, this is called to have it deployed normally
            print(f"No files have been found in the deploy folder.")
            print(f"Deploying as normal...")
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
    elif args.target == "docs":
        subprocess.run(["doxygen", "doxygen.conf"])


if __name__ == "__main__":
    main()
